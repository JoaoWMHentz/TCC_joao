#!/usr/bin/env python3
"""
pocket_explorer.py — exploração em "desbaste", como o pocketing de uma CNC.

Estratégia:
  CANTO       acha a borda externa no próprio /map por visão computacional
              (pocket.find_outer_corners): preenche os buracos do espaço livre
              (prateleiras somem), simplifica o contorno externo em polígono e
              fica com os cantos convexos com parede dos dois lados e sobre o
              fecho convexo. Vai pelo Nav2 ao canto mais próximo, já virado
              com a parede à direita. Sem canto visível: vai a uma fronteira
              (BFS) para revelar mais mapa e tenta de novo.
  NÍVEL 0     CONTORNO: a partir do canto, segue a parede externa (mesmo
              seguidor do wall_bfs_explorer) até voltar ao mesmo ponto. Se a
              volta mesmo assim foi em torno de uma ilha — percebido pelo
              sentido da volta — procura outro canto (até `contour_attempts`).
  NÍVEL 1..N  PASSES: cada passe é um offset de tudo o que o robô já
              percorreu. A região ainda não varrida é a parte alcançável do
              mapa a mais de `level_spacing` do caminho percorrido (e a
              `pass_clearance` de obstáculos e do desconhecido); a borda dela
              é o próximo passe, seguido pelo Nav2 (navigate_through_poses)
              mantendo o já feito do lado da parede. O passe entra no caminho
              percorrido, a região encolhe e o próximo é o offset dele, até
              chegar ao centro (ver pocket.py).
  ACABAMENTO  sem região para desbastar, a BFS visita as fronteiras que
              sobraram (cada chegada pode abrir área nova -> mais passes).
  FIM         nenhuma fronteira alcançável: (opcional) volta ao ponto de
              partida e encerra.

Entradas:  /scan, /map, TF map -> base_link e base_link -> frame do LiDAR
Saídas:    /cmd_vel (no contorno), actions navigate_to_pose e
           navigate_through_poses (Nav2)

Uso:
  ros2 launch navigation explore.launch.py strategy:=pocket use_sim_time:=true
"""

import math
import signal
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
import numpy as np
from pocket import (cell_to_world, find_outer_corners, is_outer_loop, path_length, plan_pass,
                    rasterize_path, region_contours, remaining_region, resample, world_to_cell)
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from wall_bfs import traversable_mask
from wall_bfs_explorer import BFS_GOAL, DONE, WALL, WallBfsExplorer

PASS, WAIT_POSE = 'pass', 'wait_pose'


class PocketExplorer(WallBfsExplorer):

    DESCRIPTION = 'desbaste: contorno + offsets'

    def __init__(self):
        super().__init__('pocket_explorer')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.level_spacing = p('level_spacing', 1.5)          # m — stepover entre passes
        self.pass_clearance = p('pass_clearance', 0.6)        # m — folga obstáculo/desconhecido
        self.waypoint_spacing = p('waypoint_spacing', 1.0)    # m — entre pontos do passe
        self.min_pass_length = p('min_pass_length', 2.0)      # m — contornos menores: ignora
        self.min_region_area = p('min_region_area', 1.0)      # m² — sobras menores: ignora
        self.blocked_radius = p('blocked_radius', 0.8)        # m — em volta de ponto inalcançável
        self.contour_attempts = p('contour_attempts', 2)      # voltas em ilha antes de desistir
        self.max_pass_failures = p('max_pass_failures', 6)    # falhas seguidas: vai ao acabamento
        # identificação da borda externa no mapa (visão computacional)
        self.corner_min_wall = p('corner_min_wall_length', 1.5)    # m — paredes do canto
        self.corner_approx_eps = p('corner_approx_eps', 0.25)      # m — Douglas-Peucker
        self.corner_hull_tol = p('corner_hull_tolerance', 0.6)     # m — até o fecho convexo
        self.corner_backing = p('corner_wall_backing', 0.7)        # fração com parede atrás
        self.corner_search_attempts = p('corner_search_attempts', 4)  # revela mapa e tenta de novo
        self.corner_skip_radius = p('corner_skip_radius', 1.5)     # m — canto já tentado

        self.passes = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.blocked: list[tuple[float, float]] = []          # pontos de passe inalcançáveis
        self.contour_done = False
        self.contour_tries = 0
        self.at_corner = False            # chegou ao canto externo (ou desistiu de achar)
        self.phase = ''                   # 'corner' | 'reveal' | '' — destino atual do Nav2
        self.corner_tries = 0
        self.corner = None                # (x, y) do canto escolhido
        self.bad_corners: list[tuple[float, float]] = []
        self.seg_points: list[tuple[float, float]] = []       # trecho atual de parede
        self.level = 0
        self.pass_failures = 0
        self.pass_points: list[tuple[float, float]] = []
        self.pass_handle = None
        self.pass_remaining = 0
        self.pass_timeout = 0.0
        self.get_logger().info('Contorno externo e depois passes em offset '
                               f'de {self.level_spacing:.1f} m do caminho percorrido')

    # ── Ciclo ────────────────────────────────────────────────────────────────

    def _tick(self):
        if self.state not in (PASS, WAIT_POSE):
            super()._tick()
            return
        pose = self._robot_pose()
        if pose is None:
            return
        self._record(pose)
        if self.state == PASS:
            self._check_pass(pose, time.monotonic())
        else:
            self._next_pass(pose)

    def _record(self, pose):
        added = super()._record(pose)
        if added and self.state == WALL:
            self.seg_points.append((pose[0], pose[1]))
        return added

    # ── Nível 0: contorno ────────────────────────────────────────────────────

    def _start_wall(self, pose, now):
        if not self.contour_done and not self.at_corner:
            self._seek_outer_corner(pose)       # primeiro acha a parede externa
            return
        super()._start_wall(pose, now)
        self.seg_points = [(pose[0], pose[1])]
        self.get_logger().info('Nível 0: contornando a parede externa')

    def _after_wall(self, pose, why):
        self._stop_robot()
        # volta fechada: voltou a um ponto já percorrido; a volta é o caminho dali até aqui
        closed = self.loop_hit is not None
        loop = self.path.xy[self.loop_hit:] if closed else self.seg_points
        if closed and not is_outer_loop(loop, self.wall.side) \
                and self.contour_tries < self.contour_attempts:
            self.contour_tries += 1
            self.get_logger().info(f'A volta ({self.seg_travel:.1f} m) foi em torno de uma ilha, '
                                   'não da parede externa; procurando outro canto')
            if self.corner is not None:
                self.bad_corners.append(self.corner)
            self.at_corner = False
            self._seek_outer_corner(pose)
            return
        if closed:
            self.get_logger().info(
                f'Contorno fechado ({self.seg_travel:.1f} m); iniciando os passes')
        else:
            self.get_logger().info(f'Contorno encerrado ({why}); iniciando os passes')
        self.contour_done = True
        self._next_pass(pose)

    def _after_frontier(self, pose):
        phase, self.phase = self.phase, ''
        if phase == 'corner':
            self.at_corner = True
            self.get_logger().info('No canto da parede externa; contornando até voltar a ele')
        if self.contour_done:
            self._next_pass(pose)
        else:
            self._start_wall(pose, time.monotonic())

    # ── Borda externa: achar um canto no mapa e ir até ele ──────────────────

    def _seek_outer_corner(self, pose):
        target = self._outer_corner(pose)
        if target is not None:
            (cx, cy), (ax, ay), yaw, walls = target
            self.corner, self.phase, self.state = (cx, cy), 'corner', BFS_GOAL
            self._send_goal(ax, ay, yaw, f'Canto da parede externa em ({cx:.1f}, {cy:.1f}), '
                                         f'paredes de {walls[0]:.1f} m e {walls[1]:.1f} m; '
                                         'indo até ele')
            return
        self.corner_tries += 1
        if self.corner_tries <= self.corner_search_attempts:
            reveal = self._bfs_target(pose)
            if reveal is not None:
                gx, gy, yaw, dist = reveal
                self.phase, self.state = 'reveal', BFS_GOAL
                self._send_goal(gx, gy, yaw, 'Borda externa ainda não identificada no mapa; '
                                             f'revelando mais (fronteira a {dist:.1f} m)')
                return
        self.get_logger().info('Nenhum canto de parede externa identificado; '
                               'seguindo a parede mais próxima')
        self.at_corner = True
        self._start_wall(pose, time.monotonic())

    def _outer_corner(self, pose):
        """Canto externo mais próximo do robô com ponto de chegada livre.

        Retorna ((x, y) do canto, (x, y) de chegada, yaw, paredes), ou None.
        """
        m = self.map
        if m is None:
            return None
        res = m.info.resolution
        origin = (m.info.origin.position.x, m.info.origin.position.y)
        grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        corners = find_outer_corners(grid, res, self.wall.wall_distance, self.corner_min_wall,
                                     self.corner_approx_eps, self.corner_hull_tol,
                                     self.corner_backing, side=self.wall.side)
        passable = traversable_mask(grid, res, self.robot_radius)
        best = None
        for k in corners:
            cx, cy = cell_to_world(*k.corner, origin, res)
            if any(math.dist((cx, cy), b) < self.corner_skip_radius for b in self.bad_corners):
                continue
            ar, ac = int(round(k.approach[0])), int(round(k.approach[1]))
            if not (0 <= ar < grid.shape[0] and 0 <= ac < grid.shape[1] and passable[ar, ac]):
                continue
            ax, ay = cell_to_world(ar, ac, origin, res)
            d = math.dist((ax, ay), pose[:2])
            if best is None or d < best[0]:
                best = (d, ((cx, cy), (ax, ay), k.yaw, k.walls_m))
        return best[1] if best else None

    def _goal_failed(self, why):
        if self.phase == 'corner' and self.corner is not None:
            self.get_logger().info(f'Não consegui chegar ao canto ({why}); tentando outro')
            self.bad_corners.append(self.corner)
            self.phase, self.goal = '', None
            pose = self._robot_pose()
            if pose is not None:
                self._seek_outer_corner(pose)
            return
        if self.phase == 'reveal' and self.goal is not None:
            self.blacklist.append(self.goal)
            self.get_logger().info(f'Fronteira ({self.goal[0]:.1f}, {self.goal[1]:.1f}) '
                                   f'descartada: {why}')
            self.phase, self.goal = '', None
            pose = self._robot_pose()
            if pose is not None:
                self._seek_outer_corner(pose)
            return
        self.phase = ''
        super()._goal_failed(why)

    def _wall_exhausted(self, pose, now, moved_at):
        """Decide se o contorno externo acabou.

        No contorno externo só para ao fechar a volta (voltar a caminho já
        percorrido), travar ou estourar o tempo: o mapa já pode ter sido visto
        do canto, então "não cresceu" não quer dizer que a parede acabou.
        """
        why = self._revisiting(pose)
        if why:
            return why
        if now - moved_at > self.stuck_time:
            return f'robô parado há {self.stuck_time:.0f} s'
        if now - self.seg_started_at > self.wall_max_time:
            return f'contorno passou de {self.wall_max_time:.0f} s'
        return None

    def _go_bfs(self, pose):
        target = self._bfs_target(pose)
        if target is None:
            if not self.contour_done:
                self.contour_done = True
                self._next_pass(pose)
            else:
                self._finish()
            return
        gx, gy, yaw, dist = target
        self._send_goal(gx, gy, yaw, f'Acabamento: fronteira a {dist:.1f} m de caminho, '
                                     f'indo para ({gx:.1f}, {gy:.1f})')
        self.state = BFS_GOAL

    # ── Níveis 1..N: passes em offset do caminho percorrido ──────────────────

    def _plan(self, pose):
        """Pontos (x, y) do próximo passe, ou None se não há o que desbastar."""
        m = self.map
        res = m.info.resolution
        origin = (m.info.origin.position.x, m.info.origin.position.y)
        shape = (m.info.height, m.info.width)
        grid = np.asarray(m.data, dtype=np.int16).reshape(shape)
        traveled = rasterize_path(self.path.xy, origin, res, shape)
        blocked = rasterize_path([], origin, res, shape)
        k = self.blocked_radius / res
        rows, cols = np.ogrid[:shape[0], :shape[1]]
        for bx, by in self.blocked:
            br, bc = world_to_cell(bx, by, origin, res)
            blocked |= (rows - br) ** 2 + (cols - bc) ** 2 <= k * k
        robot = world_to_cell(pose[0], pose[1], origin, res)
        region = remaining_region(grid, res, robot, traveled, self.level_spacing,
                                  self.pass_clearance, blocked, self.min_region_area)
        contours = region_contours(region, int(self.min_pass_length / res))
        line = plan_pass(contours, region, robot, self.wall.side)
        if line is None:
            return None
        pts = resample(line, self.waypoint_spacing / res)
        return [cell_to_world(r, c, origin, res) for r, c in pts]

    def _next_pass(self, pose):
        points = self._plan(pose) if self.map is not None else None
        if points is None:
            self.get_logger().info(f'Desbaste concluído em {self.level} nível(is); '
                                   'buscando fronteiras restantes (acabamento)')
            self._go_bfs(pose)
            return
        if len(points) > 1 and math.dist(points[0], pose[:2]) < 0.3:
            points = points[1:]                 # o primeiro ponto é onde o robô já está
        self.level += 1
        length = path_length(np.asarray([pose[:2], *points]))
        self.get_logger().info(f'Nível {self.level}: passe de {length:.1f} m '
                               f'({len(points)} pontos)')
        self._send_pass(points, length)

    def _send_pass(self, points, length):
        goal = NavigateThroughPoses.Goal()
        stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            nx, ny = points[i + 1] if i + 1 < len(points) else (x, y)
            px, py = points[i - 1] if i > 0 else (x, y)
            yaw = math.atan2(ny - y, nx - x) if i + 1 < len(points) else math.atan2(y - py, x - px)
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = self.global_frame, stamp
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.poses.append(ps)
        self.state = PASS
        self.pass_points = points
        self.pass_remaining = len(points)
        self.pass_handle = None
        self.sending = True
        now = time.monotonic()
        self.goal_sent_at = now
        self.pass_timeout = max(self.goal_timeout, 60.0 + length / 0.15)
        pose = self._robot_pose()
        if pose:
            self.moved_at = (pose[0], pose[1], now)
        self.passes.send_goal_async(goal, feedback_callback=self._on_pass_feedback) \
            .add_done_callback(self._on_pass_response)

    def _on_pass_feedback(self, msg):
        self.pass_remaining = int(msg.feedback.number_of_poses_remaining)

    def _on_pass_response(self, future):
        self.sending = False
        handle = future.result()
        if not handle.accepted:
            self._pass_failed('recusado pelo Nav2')
            return
        self.pass_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, h=handle: self._on_pass_result(h, f.result().status))

    def _check_pass(self, pose, now):
        if self.sending or self.pass_handle is None:
            return
        x, y, _ = pose
        mx, my, mt = self.moved_at
        if math.hypot(x - mx, y - my) > 0.15:
            self.moved_at = (x, y, now)
        elif now - mt > self.progress_timeout:
            self._cancel_pass(f'robô sem avançar há {self.progress_timeout:.0f} s')
        elif now - self.goal_sent_at > self.pass_timeout:
            self._cancel_pass(f'passe passou de {self.pass_timeout:.0f} s')

    def _cancel_pass(self, why):
        handle, self.pass_handle = self.pass_handle, None   # ignora o resultado do cancelamento
        if handle is not None:
            handle.cancel_goal_async()
        self._pass_failed(why)

    def _on_pass_result(self, handle, status):
        if handle is not self.pass_handle:
            return
        self.pass_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.pass_failures = 0
            self._continue_passes()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Passe cancelado por outro cliente; encerrando a exploração.')
            self._done()
        else:
            self._pass_failed('o Nav2 não conseguiu seguir o passe')

    def _pass_failed(self, why):
        # o ponto que o robô não alcançou fica fora dos próximos passes
        i = min(max(len(self.pass_points) - self.pass_remaining, 0), len(self.pass_points) - 1)
        if self.pass_points:
            bx, by = self.pass_points[i]
            self.blocked.append((bx, by))
            self.get_logger().info(f'Passe do nível {self.level} interrompido ({why}); '
                                   f'ponto ({bx:.1f}, {by:.1f}) descartado')
        self.pass_failures += 1
        if self.pass_failures >= self.max_pass_failures:
            self.get_logger().info(f'{self.pass_failures} passes seguidos falharam; '
                                   'indo para o acabamento')
            self.pass_failures = 0
            pose = self._robot_pose()
            if pose is not None:
                self._go_bfs(pose)
            return
        self._continue_passes()

    def _continue_passes(self):
        pose = self._robot_pose()
        if pose is not None:
            self._next_pass(pose)
        else:
            self.state = WAIT_POSE

    def _finish(self):
        self.get_logger().info(f'Desbaste: {self.level} nível(is), '
                               f'{self.path.length:.0f} m percorridos')
        super()._finish()

    def shutdown(self):
        if self.pass_handle is not None:
            future = self.pass_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        super().shutdown()


def main():
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)

    def on_sigterm(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)
    node = PocketExplorer()
    try:
        while rclpy.ok() and node.state != DONE:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.get_logger().info('Exploração interrompida.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
