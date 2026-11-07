#!/usr/bin/env python3
"""
wall_bfs_explorer.py — exploração automática simples: seguir parede + BFS.

Estratégia:
  SEGUIR PAREDE  o robô anda mantendo a parede de um lado (regra da mão
                 direita/esquerda), comandando /cmd_vel direto a partir do
                 /scan. Contorna o ambiente revelando o mapa pelo caminho.
  BFS            quando seguir a parede deixa de render — voltou a passar,
                 no mesmo sentido, por um lugar já percorrido (o caminho
                 fica registrado), o mapa parou de crescer,
                 o robô travou ou estourou o tempo do trecho — uma busca em
                 largura no /map acha a fronteira alcançável mais próxima
                 pelo caminho (ver wall_bfs.py) e o Nav2 leva o robô até lá.
                 Ao chegar, volta a seguir parede.
  FIM            a BFS não acha fronteira alcançável em
                 `empty_rounds_to_finish` tentativas seguidas: (opcional)
                 volta ao ponto de partida e encerra.

Entradas:  /scan, /map, TF map -> base_link e base_link -> frame do LiDAR
Saídas:    /cmd_vel (ao seguir parede), action navigate_to_pose (Nav2)

Ctrl+C / SIGTERM para o robô (Twist zero e cancela o destino) antes de sair.

Uso:
  ros2 launch navigation explore.launch.py strategy:=wall_bfs use_sim_time:=true
"""

import math
import signal
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from wall_bfs import (PathMemory, WallParams, bfs_to_frontier, free_disk, frontier_targets,
                      scan_view, traversable_mask, wall_follow_command)

MAP_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

WAIT, WALL, BFS_GOAL, RETURNING, DONE = 'wait', 'wall', 'bfs_goal', 'returning', 'done'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class WallBfsExplorer(Node):

    DESCRIPTION = 'seguir parede + BFS'

    def __init__(self, node_name: str = 'wall_bfs_explorer'):
        super().__init__(node_name)
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.global_frame = p('global_frame', 'map')
        self.base_frame = p('robot_base_frame', 'base_link')
        self.wall = WallParams(
            side=p('wall_side', 'right'),
            wall_distance=p('wall_distance', 0.70),
            front_clearance=p('front_clearance', 0.75),
            wall_lost=p('wall_lost_distance', 1.40),
            linear_speed=p('linear_speed', 0.8),
            min_linear_speed=p('min_linear_speed', 0.12),
            max_angular_speed=p('max_angular_speed', 0.8),
            kp=p('kp', 1.5),
            ka=p('ka', 1.0),
            corner_turn_rate=p('corner_turn_rate', 0.6),
            slow_zone=p('slow_zone', 1.0),
        )
        self.corner_time = p('corner_time', 4.0)                  # s
        self.robot_radius = p('robot_radius', 0.33)               # m
        self.min_frontier_size = p('min_frontier_size', 0.3)      # m
        self.loop_min_travel = p('loop_min_travel', 4.0)          # m
        self.loop_close_radius = p('loop_close_radius', 0.6)      # m
        self.revisit_travel = p('revisit_travel', 1.0)            # m sobre o já percorrido
        self.revisit_heading = math.radians(p('revisit_heading', 60.0))
        self.stagnation_time = p('stagnation_time', 25.0)         # s
        self.stagnation_gain = p('stagnation_gain', 0.5)          # m² de mapa novo
        self.wall_max_time = p('wall_max_time', 180.0)            # s
        self.stuck_time = p('stuck_time', 10.0)                   # s
        self.blacklist_radius = p('blacklist_radius', 0.6)        # m
        self.progress_timeout = p('progress_timeout', 30.0)       # s
        self.goal_timeout = p('goal_timeout', 180.0)              # s
        self.empty_rounds_to_finish = p('empty_rounds_to_finish', 3)
        self.return_to_start = p('return_to_start', True)
        rate = p('control_rate', 10.0)                            # Hz
        self.path = PathMemory(p('record_spacing', 0.05))         # caminho percorrido

        self.create_subscription(OccupancyGrid, p('map_topic', '/map'), self._on_map, MAP_QOS)
        self.create_subscription(LaserScan, p('scan_topic', '/scan'), self._on_scan,
                                 qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic', '/cmd_vel'), 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map: OccupancyGrid | None = None
        self.scan: LaserScan | None = None
        self.scan_yaw: float | None = None     # yaw do LiDAR no base_link
        self.state = WAIT
        self.start_pose = None
        self.blacklist: list[tuple[float, float]] = []
        self.empty_rounds = 0
        self.goals_reached = 0
        self.started_at = time.monotonic()
        # trecho atual de seguir parede
        self.seg_started_at = 0.0
        self.seg_travel = 0.0
        self.last_pose = None
        self.last_wall_at = 0.0
        self.known_area = 0.0
        self.known_checked_at = 0.0
        self.moved_at = (0.0, 0.0, 0.0)        # x, y, instante do último deslocamento
        self.wall_state = ''
        self.revisit_from = None               # (distância, índice) ao entrar no já percorrido
        self.loop_hit = None                   # índice do ponto revisitado que encerrou o trecho
        # destino do Nav2
        self.goal = None
        self.goal_handle = None
        self.goal_sent_at = 0.0
        self.sending = False

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'Exploração ({self.DESCRIPTION}) iniciada; '
                               'aguardando mapa, scan e Nav2...')

    # ── Entradas ─────────────────────────────────────────────────────────────

    def _on_map(self, msg):
        self.map = msg

    def _on_scan(self, msg):
        self.scan = msg
        if self.scan_yaw is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id,
                                                     rclpy.time.Time())
                self.scan_yaw = yaw_of(tf.transform.rotation)
            except tf2_ros.TransformException:
                pass

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame,
                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t = tf.transform.translation
        return t.x, t.y, yaw_of(tf.transform.rotation)

    def _record(self, pose):
        """Registra a pose no caminho percorrido; True se entrou um ponto novo."""
        return self.path.add(*pose)

    def _known_area(self):
        m = self.map
        known = np.count_nonzero(np.asarray(m.data, dtype=np.int16) >= 0)
        return known * m.info.resolution ** 2

    # ── Ciclo ────────────────────────────────────────────────────────────────

    def _tick(self):
        if self.state == DONE:
            return
        pose = self._robot_pose()
        now = time.monotonic()
        if pose is not None:
            self._record(pose)
        if self.state == WAIT:
            if (self.map is None or self.scan is None or self.scan_yaw is None
                    or pose is None or not self.nav.server_is_ready()):
                return
            if self.start_pose is None:
                self.start_pose = pose
            self._start_wall(pose, now)
            return
        if pose is None:
            if self.state == WALL:
                self._stop_robot()          # sem localização: não anda às cegas
            return
        if self.state == WALL:
            self._wall_step(pose, now)
        elif self.state in (BFS_GOAL, RETURNING):
            self._check_goal(pose, now)

    # ── Seguir parede ────────────────────────────────────────────────────────

    def _start_wall(self, pose, now):
        self.state = WALL
        self.seg_started_at = now
        self.seg_travel = 0.0
        self.last_pose = pose
        self.last_wall_at = 0.0           # começa procurando parede, sem "quina"
        self.known_area = self._known_area()
        self.known_checked_at = now
        self.moved_at = (pose[0], pose[1], now)
        self.wall_state = ''
        self.revisit_from = None
        self.loop_hit = None
        self.get_logger().info(f'Seguindo a parede ({self.wall.side})')

    def _wall_step(self, pose, now):
        x, y, _ = pose
        self.seg_travel += math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        self.last_pose = pose
        mx, my, mt = self.moved_at
        if math.hypot(x - mx, y - my) > 0.1:
            self.moved_at = (x, y, now)

        why = self._wall_exhausted(pose, now, mt)
        if why:
            self._stop_robot()
            self._after_wall(pose, why)
            return

        sc = self.scan
        view = scan_view(sc.ranges, sc.angle_min, sc.angle_increment, sc.range_max,
                         self.wall.side, self.scan_yaw)
        recently = now - self.last_wall_at < self.corner_time
        v, w, st = wall_follow_command(view, self.wall, recently)
        if st in ('follow', 'turn'):
            self.last_wall_at = now
        if st != self.wall_state:
            self.get_logger().debug(f'seguidor: {st}')
            self.wall_state = st
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    def _revisiting(self, pose):
        """Motivo se o robô está há `revisit_travel` m andando, no mesmo
        sentido, sobre caminho já percorrido (deste trecho, a mais de
        `loop_min_travel` m para trás, ou de qualquer trecho anterior) — ou None.
        Guarda em loop_hit o índice do primeiro ponto revisitado."""
        hit = self.path.revisit(*pose, self.loop_close_radius, self.loop_min_travel,
                                self.revisit_heading)
        if hit is None:
            self.revisit_from = None
            return None
        if self.revisit_from is None:
            self.revisit_from = (self.path.length, hit)
        start, first = self.revisit_from
        if self.path.length - start < self.revisit_travel:
            return None
        self.loop_hit = first
        px, py = self.path.xy[first]
        return (f'voltou a caminho já percorrido em ({px:.1f}, {py:.1f}) '
                f'após {self.seg_travel:.1f} m de parede')

    def _wall_exhausted(self, pose, now, moved_at):
        """Motivo para largar a parede e chamar a BFS, ou None."""
        why = self._revisiting(pose)
        if why:
            return why
        if now - moved_at > self.stuck_time:
            return f'robô parado há {self.stuck_time:.0f} s'
        if now - self.seg_started_at > self.wall_max_time:
            return f'trecho passou de {self.wall_max_time:.0f} s'
        if now - self.known_checked_at > self.stagnation_time:
            area = self._known_area()
            gain, self.known_area, self.known_checked_at = area - self.known_area, area, now
            if gain < self.stagnation_gain:
                return f'mapa não cresceu em {self.stagnation_time:.0f} s'
        return None

    def _after_wall(self, pose, why):
        """Trecho de parede encerrado (why = motivo). Aqui: vai para a BFS."""
        self.get_logger().info(f'Parede: {why}; buscando fronteira (BFS)')
        self._go_bfs(pose)

    # ── BFS ──────────────────────────────────────────────────────────────────

    def _go_bfs(self, pose):
        target = self._bfs_target(pose)
        if target is None:
            self.empty_rounds += 1
            if self.empty_rounds >= self.empty_rounds_to_finish:
                self._finish()
            else:
                self.get_logger().info('Nenhuma fronteira alcançável; seguindo a parede mais um pouco')
                self._start_wall(pose, time.monotonic())
            return
        self.empty_rounds = 0
        gx, gy, yaw, dist = target
        self._send_goal(gx, gy, yaw, f'BFS: fronteira a {dist:.1f} m de caminho, '
                                     f'indo para ({gx:.1f}, {gy:.1f})')
        self.state = BFS_GOAL

    def _bfs_target(self, pose):
        """(x, y, yaw, distância pelo caminho) da fronteira mais próxima, ou None."""
        m = self.map
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        start = (int((pose[1] - oy) / res), int((pose[0] - ox) / res))
        if not (0 <= start[0] < grid.shape[0] and 0 <= start[1] < grid.shape[1]):
            return None
        passable = traversable_mask(grid, res, self.robot_radius)
        passable |= free_disk(grid, start, int(math.ceil((self.robot_radius + 0.15) / res)))
        targets = frontier_targets(grid, max(1, int(round(self.min_frontier_size / res))))
        # fronteiras já descartadas saem da busca
        for bx, by in self.blacklist:
            c, r = int((bx - ox) / res), int((by - oy) / res)
            k = int(math.ceil(self.blacklist_radius / res))
            targets[max(0, r - k):r + k + 1, max(0, c - k):c + k + 1] = False
        result = bfs_to_frontier(passable, targets, start)
        if result is None:
            return None
        r, c = result.goal
        gx, gy = ox + (c + 0.5) * res, oy + (r + 0.5) * res
        # orientação final: direção dos últimos passos do caminho (de frente pro desconhecido)
        pr, pc = result.path[max(0, len(result.path) - 6)]
        yaw = math.atan2(r - pr, c - pc) if (r, c) != (pr, pc) else pose[2]
        return gx, gy, yaw, result.length_cells * res

    # ── Nav2 ─────────────────────────────────────────────────────────────────

    def _send_goal(self, x, y, yaw, why):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info(why)
        self.goal, self.sending, self.goal_handle = (x, y), True, None
        self.goal_sent_at = time.monotonic()
        pose = self._robot_pose()
        if pose:
            self.moved_at = (pose[0], pose[1], self.goal_sent_at)
        self.nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self.sending = False
        handle = future.result()
        if not handle.accepted:
            self._goal_failed('recusado pelo Nav2')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, h=handle: self._on_result(h, f.result().status))

    def _check_goal(self, pose, now):
        if self.sending or self.goal is None:
            return
        x, y, _ = pose
        mx, my, mt = self.moved_at
        if math.hypot(x - mx, y - my) > 0.15:
            self.moved_at = (x, y, now)
        elif now - mt > self.progress_timeout:
            self._cancel_and_fail(f'robô sem avançar há {self.progress_timeout:.0f} s')
        elif now - self.goal_sent_at > self.goal_timeout:
            self._cancel_and_fail(f'mais de {self.goal_timeout:.0f} s sem chegar')

    def _cancel_and_fail(self, why):
        handle, self.goal_handle = self.goal_handle, None   # ignora o resultado do cancelamento
        if handle is not None:
            handle.cancel_goal_async()
        self._goal_failed(why)

    def _goal_failed(self, why):
        if self.state == RETURNING:
            self.get_logger().info(f'Não consegui voltar ao ponto de partida ({why}).')
            self._done()
            return
        if self.goal is not None:
            self.blacklist.append(self.goal)
            self.get_logger().info(f'Fronteira ({self.goal[0]:.1f}, {self.goal[1]:.1f}) '
                                   f'descartada: {why}')
        self.goal = None
        pose = self._robot_pose()
        if pose is not None:
            self._go_bfs(pose)
        else:
            self.state = WAIT

    def _on_result(self, handle, status):
        if handle is not self.goal_handle:
            return                       # resultado de um destino antigo/cancelado
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == RETURNING:
                self.get_logger().info('De volta ao ponto de partida.')
                self._done()
                return
            self.goals_reached += 1
            # se ainda houver fronteira ali, ela não se revela (ex.: vidro): não volta nela
            self.blacklist.append(self.goal)
            self.goal = None
            pose = self._robot_pose()
            if pose is not None:
                self._after_frontier(pose)
            else:
                self.state = WAIT
        elif status == GoalStatus.STATUS_CANCELED:
            # cancelado por outro cliente (ex.: interface web): para de explorar
            self.get_logger().info('Destino cancelado por outro cliente; encerrando a exploração.')
            self._done()
        else:
            self._goal_failed('o Nav2 não conseguiu chegar')

    def _after_frontier(self, pose):
        """Chegou à fronteira da BFS. Aqui: volta a seguir parede."""
        self._start_wall(pose, time.monotonic())

    # ── Fim ──────────────────────────────────────────────────────────────────

    def _finish(self):
        mins = (time.monotonic() - self.started_at) / 60
        self.get_logger().info(f'Exploração concluída: nenhuma fronteira alcançável '
                               f'({self.goals_reached} fronteira(s) visitada(s) em {mins:.1f} min).')
        if self.return_to_start and self.start_pose is not None:
            x, y, yaw = self.start_pose
            self.state = RETURNING
            self._send_goal(x, y, yaw, 'Voltando ao ponto de partida')
        else:
            self._done()

    def _done(self):
        self._stop_robot()
        self.state = DONE
        self.get_logger().info('Exploração encerrada.')

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def shutdown(self):
        """Para o robô e cancela o destino atual antes de sair."""
        if self.state == WALL:
            self._stop_robot()
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main():
    # Sem o tratador de sinais do rclpy: no Ctrl+C ainda precisamos do
    # contexto ativo para parar o robô e cancelar o destino no Nav2.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)

    def on_sigterm(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)
    node = WallBfsExplorer()
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
