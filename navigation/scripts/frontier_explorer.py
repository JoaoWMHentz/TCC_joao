#!/usr/bin/env python3
"""
frontier_explorer.py — exploração automática baseada em fronteiras.

Durante o mapeamento, escolhe sozinho para onde o robô vai: encontra as
fronteiras do mapa (borda entre o espaço livre já visto e o desconhecido,
ver frontiers.py), escolhe a melhor e manda o Nav2 até ela. Repete até não
sobrar fronteira alcançável; então (opcional) volta ao ponto de partida e
encerra.

Ciclo (timer, `update_period` s):
  1. calcula as fronteiras do /map (RTAB-Map);
  2. descarta as que estão na lista negra (destinos que falharam ou que
     já foram visitados sem revelar o desconhecido, ex.: atrás de vidro);
  3. nota = peso_dist * distância - peso_tam * tamanho; menor nota vence;
  4. navegando: troca de destino se a fronteira atual sumiu (já foi
     revelada); cancela e põe na lista negra se o robô não avançar em
     `progress_timeout` s ou se o destino passar de `goal_timeout` s.

Entradas:  /map (OccupancyGrid), /global_costmap/costmap, TF map -> base_link
Saídas:    action navigate_to_pose (Nav2)

Ctrl+C (SIGINT) cancela o destino atual antes de sair, para o robô parar.

Uso:
  ros2 launch navigation explore.launch.py use_sim_time:=true
"""

import math
import signal
import sys
import time

from action_msgs.msg import GoalStatus
from frontiers import find_frontiers, goal_candidates, min_distance
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
import tf2_ros

MAP_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
COSTMAP_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
COST_BLOCKED = 90          # costmap (0..100): >= isto o centro do robô não cabe
EMPTY_ROUNDS_TO_FINISH = 3  # ciclos seguidos sem fronteira para dar por concluído


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        self.global_frame = p('global_frame', 'map').value
        self.base_frame = p('robot_base_frame', 'base_link').value
        self.min_frontier_size = p('min_frontier_size', 0.5).value      # m
        self.distance_weight = p('distance_weight', 1.0).value
        self.size_weight = p('size_weight', 0.5).value
        self.blacklist_radius = p('blacklist_radius', 0.6).value        # m
        self.frontier_gone_dist = p('frontier_gone_distance', 0.5).value  # m
        self.min_goal_distance = p('min_goal_distance', 0.4).value      # m
        self.progress_dist = p('progress_distance', 0.15).value         # m
        self.progress_timeout = p('progress_timeout', 30.0).value       # s
        self.goal_timeout = p('goal_timeout', 180.0).value              # s
        self.return_to_start = p('return_to_start', True).value
        self.update_period = p('update_period', 1.0).value              # s

        self.map: OccupancyGrid | None = None
        self.costmap: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, p('map_topic', '/map').value,
                                 self._on_map, MAP_QOS)
        self.create_subscription(OccupancyGrid,
                                 p('costmap_topic', '/global_costmap/costmap').value,
                                 self._on_costmap, COSTMAP_QOS)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.blacklist: list[tuple[float, float]] = []
        self.start_pose: tuple[float, float, float] | None = None
        self.goal: tuple[float, float] | None = None   # destino atual (x, y)
        self.goal_handle = None
        self.goal_sent_at = 0.0
        self.sending = False             # aguardando o Nav2 aceitar
        self.preempting = False          # cancelamos de propósito (troca de destino)
        self.returning = False           # voltando ao início
        self.finished = False
        self.empty_rounds = 0
        self.last_progress = (0.0, 0.0, 0.0)  # x, y, instante
        self.goals_reached = 0
        self.started_at = time.monotonic()

        self.create_timer(self.update_period, self._tick)
        self.get_logger().info('Exploração automática iniciada; aguardando mapa e Nav2...')

    # ── Entradas ─────────────────────────────────────────────────────────────

    def _on_map(self, msg):
        self.map = msg

    def _on_costmap(self, msg):
        self.costmap = msg

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame,
                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _cost_at(self, x, y):
        """Custo do costmap global em (x, y); -1 se desconhecido/fora/sem costmap."""
        cm = self.costmap
        if cm is None:
            return -1
        res, o = cm.info.resolution, cm.info.origin.position
        col, row = int((x - o.x) / res), int((y - o.y) / res)
        if not (0 <= col < cm.info.width and 0 <= row < cm.info.height):
            return -1
        return cm.data[row * cm.info.width + col]

    def _blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius
                   for bx, by in self.blacklist)

    def _add_blacklist(self, x, y, why):
        self.blacklist.append((x, y))
        self.get_logger().info(f'Destino ({x:.1f}, {y:.1f}) descartado: {why}')

    # ── Fronteiras ───────────────────────────────────────────────────────────

    def _frontiers(self):
        m = self.map
        grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        res = m.info.resolution
        min_cells = max(1, int(round(self.min_frontier_size / res)))
        return find_frontiers(grid, res, (m.info.origin.position.x, m.info.origin.position.y),
                              min_cells), res

    def _choose(self, frontiers, res, rx, ry):
        """(nota, x, y) do melhor destino, ou None."""
        best = None
        for f in frontiers:
            for gx, gy in goal_candidates(f)[:50]:
                if self._blacklisted(gx, gy) or self._cost_at(gx, gy) >= COST_BLOCKED:
                    continue
                d = math.hypot(gx - rx, gy - ry)
                if d < self.min_goal_distance:
                    continue
                score = self.distance_weight * d - self.size_weight * f.length_m(res)
                if best is None or score < best[0]:
                    best = (score, float(gx), float(gy))
                break   # uma célula por fronteira: a mais central que estiver livre
        return best

    # ── Ciclo ────────────────────────────────────────────────────────────────

    def _tick(self):
        if self.finished or self.sending or self.returning:
            return
        if self.preempting and self.goal is not None:
            return                       # cancelamento em andamento; espera o resultado
        pose = self._robot_pose()
        if self.map is None or pose is None:
            return
        if not self.nav.server_is_ready():
            return
        rx, ry, ryaw = pose
        now = time.monotonic()
        if self.start_pose is None:
            self.start_pose = pose
            self.last_progress = (rx, ry, now)

        frontiers, res = self._frontiers()
        live = [f for f in frontiers
                if not all(self._blacklisted(x, y) for x, y in f.cells[::max(1, f.size // 10)])]

        if self.goal is not None:
            self._check_current_goal(live, rx, ry, now)
            return

        best = self._choose(live, res, rx, ry)
        if best is None:
            self.empty_rounds += 1
            if self.empty_rounds >= EMPTY_ROUNDS_TO_FINISH:
                self._finish()
            return
        self.empty_rounds = 0
        _, gx, gy = best
        self._send_goal(gx, gy, math.atan2(gy - ry, gx - rx),
                        f'indo para a fronteira em ({gx:.1f}, {gy:.1f}); '
                        f'{len(live)} fronteira(s) restante(s)')

    def _check_current_goal(self, live, rx, ry, now):
        gx, gy = self.goal
        lx, ly, lt = self.last_progress
        if math.hypot(rx - lx, ry - ly) > self.progress_dist:
            self.last_progress = (rx, ry, now)
        elif now - lt > self.progress_timeout:
            self._add_blacklist(gx, gy, f'robô sem avançar há {self.progress_timeout:.0f} s')
            self._cancel_goal()
            return
        if now - self.goal_sent_at > self.goal_timeout:
            self._add_blacklist(gx, gy, f'mais de {self.goal_timeout:.0f} s sem chegar')
            self._cancel_goal()
            return
        cells = np.concatenate([f.cells for f in live]) if live else np.empty((0, 2))
        if min_distance(cells, gx, gy) > self.frontier_gone_dist:
            self.get_logger().info(f'Fronteira em ({gx:.1f}, {gy:.1f}) já foi revelada; '
                                   'escolhendo outra')
            self._cancel_goal()

    # ── Nav2 ─────────────────────────────────────────────────────────────────

    def _send_goal(self, x, y, yaw, why):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        q = yaw_to_quaternion(yaw)
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = q
        self.get_logger().info(why[0].upper() + why[1:])
        self.goal, self.sending, self.preempting = (x, y), True, False
        self.goal_sent_at = time.monotonic()
        pose = self._robot_pose()
        if pose:
            self.last_progress = (pose[0], pose[1], self.goal_sent_at)
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self.sending = False
        handle = future.result()
        if not handle.accepted:
            if self.goal and not self.returning:
                self._add_blacklist(*self.goal, 'recusado pelo Nav2')
            self.goal = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, h=handle: self._on_result(h, f.result().status))

    def _on_result(self, handle, status):
        if handle is not self.goal_handle:
            return                       # resultado de um destino antigo
        self.goal_handle = None
        goal, self.goal = self.goal, None
        if self.returning:
            self.get_logger().info('De volta ao ponto de partida.' if status ==
                                   GoalStatus.STATUS_SUCCEEDED else
                                   'Não consegui voltar ao ponto de partida.')
            self._done()
            return
        if goal is None:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            # chegou: se ali ainda houver fronteira, ela não se revela (ex.: vidro)
            self.blacklist.append(goal)
        elif status == GoalStatus.STATUS_CANCELED and self.preempting:
            pass                         # troca de destino pedida por nós
        elif status == GoalStatus.STATUS_CANCELED:
            # cancelado por outro cliente (ex.: interface web): para de explorar
            self.get_logger().info('Destino cancelado por outro cliente; encerrando a exploração.')
            self._done()
        else:
            self._add_blacklist(*goal, 'o Nav2 não conseguiu chegar')

    def _cancel_goal(self):
        self.preempting = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        else:
            self.goal = None

    # ── Fim ──────────────────────────────────────────────────────────────────

    def _finish(self):
        mins = (time.monotonic() - self.started_at) / 60
        self.get_logger().info(f'Exploração concluída: nenhuma fronteira alcançável '
                               f'({self.goals_reached} destino(s) em {mins:.1f} min).')
        if self.return_to_start and self.start_pose is not None:
            x, y, yaw = self.start_pose
            self.returning = True
            self._send_goal(x, y, yaw, 'voltando ao ponto de partida')
        else:
            self._done()

    def _done(self):
        self.finished = True
        self.get_logger().info('Exploração encerrada.')

    def shutdown(self):
        """Cancela o destino atual (o robô para) antes de sair."""
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main():
    # Sem o tratador de sinais do rclpy: no Ctrl+C ainda precisamos do
    # contexto ativo para cancelar o destino no Nav2.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)

    def on_sigterm(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)
    node = FrontierExplorer()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info('Exploração interrompida.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
