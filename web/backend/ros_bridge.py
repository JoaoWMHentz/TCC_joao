"""
ros_bridge.py — nó ROS 2 do backend web.

Assina o estado do robô e o repassa à interface (via `emit`), e executa os
comandos do operador:

  /map (OccupancyGrid)          -> imagem PNG do mapa
  TF map -> base_link           -> pose do robô
  /plan, /scan                  -> camadas do mapa (caminho planejado, laser)
  /diagnostics                  -> saúde do hardware
  NavigateToPose (action)       <- ir até um destino; rotas = sequência deles
  /initialpose                  <- corrigir a localização manualmente
  /cmd_vel                      <- teleoperação

Cada navegação gera um registro em navigation_log (ver db.py) com duração,
distância percorrida, recuperações do Nav2, menor distância a obstáculos e
erro final em relação ao destino.
"""

from __future__ import annotations

import base64
import hashlib
import io
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
import rclpy
import rclpy.signals
import tf2_ros
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from PIL import Image
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan

import config
import db

log = logging.getLogger('ros_bridge')

MAP_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
BODY_RADIUS_M = 0.245         # raio do corpo (LiDAR no centro do corpo)
OVERLAY_PERIOD_S = 0.5        # caminho e laser: 2 Hz bastam para a interface
OVERLAY_MAX_POINTS = 360
RESULT_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def occupancy_to_png_b64(msg: OccupancyGrid) -> str:
    """OccupancyGrid -> PNG em tons de cinza (livre branco, ocupado preto)."""
    data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    img = np.full(data.shape, 205, dtype=np.uint8)          # desconhecido
    known = data >= 0
    img[known] = (255 - data[known] * 255 // 100).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(np.flipud(img), mode='L').save(buf, format='PNG')
    return base64.b64encode(buf.getvalue()).decode()


@dataclass
class Mission:
    """Uma navegação em andamento (destino único ou rota)."""
    kind: str
    label: str
    targets: list[dict]
    map_name: str | None
    log_id: int | None = None
    index: int = 0
    started: float = field(default_factory=time.monotonic)
    distance: float = 0.0
    recoveries: int = 0
    min_range: float = math.inf
    last_pose: tuple[float, float] | None = None
    goal_handle: object = None
    cancelled: bool = False


class RosBridge(Node):

    def __init__(self, emit: Callable[[str, dict], None]):
        super().__init__('web_bridge', parameter_overrides=[
            Parameter('use_sim_time', value=config.SIM)])
        self._emit = emit
        self._lock = threading.Lock()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map: dict | None = None
        self._pending_map: OccupancyGrid | None = None
        self._map_digest: bytes | None = None
        self._pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'available': False}
        self._hardware: dict[str, dict] = {}
        self._plan_t = self._scan_t = 0.0
        self._mission: Mission | None = None
        self.map_name: str | None = None   # mapa da sessão ativa (definido pelo app)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(OccupancyGrid, 'map', self._on_map, MAP_QOS)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, 'diagnostics', self._on_diagnostics, 10)

        self.create_timer(0.2, self._pose_tick)
        self.create_timer(1.0, self._map_tick)

    # ── Mapa ─────────────────────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid):
        self._pending_map = msg  # processado a 1 Hz pelo timer

    def _map_tick(self):
        msg, self._pending_map = self._pending_map, None
        if msg is None:
            return
        info = msg.info
        digest = hashlib.blake2b(bytes(np.asarray(msg.data, dtype=np.int8)), digest_size=16)
        digest.update(f'{info.width}x{info.height}{info.origin.position}'.encode())
        if digest.digest() == self._map_digest:
            return
        self._map_digest = digest.digest()
        self._map = {
            'png_b64': occupancy_to_png_b64(msg),
            'width': info.width, 'height': info.height, 'resolution': info.resolution,
            'origin_x': info.origin.position.x, 'origin_y': info.origin.position.y,
        }
        self._emit('map_update', {k: v for k, v in self._map.items() if k != 'png_b64'})

    def get_map(self) -> dict | None:
        return self._map

    def clear_map(self):
        self._map = self._pending_map = self._map_digest = None

    # ── Pose ─────────────────────────────────────────────────────────────────

    def _pose_tick(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            pose = {'x': t.x, 'y': t.y, 'theta': yaw_from_quaternion(tf.transform.rotation),
                    'available': True}
        except tf2_ros.TransformException:
            pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'available': False}
        self._pose = pose
        self._emit('pose_update', pose)

        mission = self._mission
        if mission is not None and pose['available']:
            here = (pose['x'], pose['y'])
            if mission.last_pose is not None:
                mission.distance += math.dist(here, mission.last_pose)
            mission.last_pose = here

    def get_pose(self) -> dict:
        return dict(self._pose)

    # ── Camadas: caminho planejado e laser ───────────────────────────────────

    def _on_plan(self, msg: Path):
        now = time.monotonic()
        if now - self._plan_t < OVERLAY_PERIOD_S:
            return
        self._plan_t = now
        step = max(1, len(msg.poses) // OVERLAY_MAX_POINTS)
        self._emit('path_update', {'points': [
            [round(p.pose.position.x, 2), round(p.pose.position.y, 2)] for p in msg.poses[::step]]})

    def _on_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        mission = self._mission
        if mission is not None and valid.any():
            mission.min_range = min(mission.min_range, float(ranges[valid].min()))

        now = time.monotonic()
        if now - self._scan_t < OVERLAY_PERIOD_S:
            return
        self._scan_t = now
        try:
            tf = self._tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        r, a = ranges[valid], angles[valid]
        step = max(1, r.size // OVERLAY_MAX_POINTS)
        r, a = r[::step], a[::step]
        # Rotação 3D completa: o LiDAR pode estar montado invertido
        q, t = tf.transform.rotation, tf.transform.translation
        px, py = r * np.cos(a), r * np.sin(a)
        mx = t.x + (1 - 2 * (q.y ** 2 + q.z ** 2)) * px + 2 * (q.x * q.y - q.z * q.w) * py
        my = t.y + 2 * (q.x * q.y + q.z * q.w) * px + (1 - 2 * (q.x ** 2 + q.z ** 2)) * py
        self._emit('scan_update', {'points': np.round(np.stack([mx, my], axis=1), 2).tolist()})

    # ── Diagnóstico do hardware ──────────────────────────────────────────────

    def _on_diagnostics(self, msg: DiagnosticArray):
        for st in msg.status:
            level = st.level if isinstance(st.level, int) else int.from_bytes(st.level, 'little')
            self._hardware[st.name] = {'level': level, 'message': st.message}
        self._emit('hardware_update', self._hardware)

    def hardware(self) -> dict:
        return dict(self._hardware)

    # ── Comandos simples ─────────────────────────────────────────────────────

    def set_initial_pose(self, x: float, y: float, theta: float):
        """Equivalente ao "2D Pose Estimate" do RViz; o RTAB-Map escuta /initialpose."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.pose.pose.orientation.w = math.cos(theta / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)

    def teleop(self, linear: float, angular: float):
        if self._mission is not None:
            self.cancel()
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self._cmd_vel_pub.publish(msg)

    # ── Missões (destino único ou rota) ──────────────────────────────────────

    def mission_status(self) -> dict | None:
        m = self._mission
        if m is None:
            return None
        return {'kind': m.kind, 'label': m.label, 'index': m.index, 'total': len(m.targets)}

    def start_mission(self, kind: str, label: str, targets: list[dict]) -> tuple[bool, str]:
        if not targets:
            return False, 'nenhum destino'
        if not self._nav.wait_for_server(timeout_sec=2.0):
            return False, 'Navegação (Nav2) não está disponível'
        with self._lock:
            if self._mission is not None:
                self._cancel_locked()
            mission = Mission(kind=kind, label=label, targets=targets, map_name=self.map_name)
            self._mission = mission
        try:
            mission.log_id = db.log_start(self.map_name, kind, label, targets)
        except Exception:
            log.exception('falha ao registrar a navegação no banco')
        self._send_leg(mission)
        return True, 'ok'

    def cancel(self):
        with self._lock:
            self._cancel_locked()
        self._cmd_vel_pub.publish(Twist())

    def _cancel_locked(self):
        mission = self._mission
        if mission is None:
            return
        mission.cancelled = True
        if mission.goal_handle is not None:
            mission.goal_handle.cancel_goal_async()

    def _goal_msg(self, target: dict) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target['x'])
        goal.pose.pose.position.y = float(target['y'])
        theta = float(target.get('theta', 0.0))
        goal.pose.pose.orientation.z = math.sin(theta / 2)
        goal.pose.pose.orientation.w = math.cos(theta / 2)
        return goal

    def _send_leg(self, mission: Mission):
        target = mission.targets[mission.index]
        self._emit_goal(mission, 'running', f'Indo para {target.get("name", "destino")}')
        leg_recoveries = [0]

        def on_feedback(fb):
            leg_recoveries[0] = int(fb.feedback.number_of_recoveries)
            remaining = float(fb.feedback.distance_remaining)
            self._emit('goal_status', {**self._goal_base(mission), 'status': 'running',
                                       'distance_remaining': round(remaining, 2)})

        def on_accepted(future):
            handle = future.result()
            if not handle.accepted:
                self._finish(mission, 'rejected')
                return
            mission.goal_handle = handle
            if mission.cancelled:
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(on_result)

        def on_result(future):
            mission.recoveries += leg_recoveries[0]
            result = RESULT_NAMES.get(future.result().status, 'aborted')
            if result == 'succeeded' and not mission.cancelled \
                    and mission.index + 1 < len(mission.targets):
                mission.index += 1
                self._send_leg(mission)
            else:
                self._finish(mission, 'canceled' if mission.cancelled else result)

        self._nav.send_goal_async(self._goal_msg(target), feedback_callback=on_feedback) \
            .add_done_callback(on_accepted)

    def _goal_base(self, mission: Mission) -> dict:
        target = mission.targets[mission.index]
        return {'kind': mission.kind, 'label': mission.label, 'index': mission.index,
                'total': len(mission.targets), 'target': target}

    def _emit_goal(self, mission: Mission, status: str, text: str):
        self._emit('goal_status', {**self._goal_base(mission), 'status': status, 'text': text})

    def _finish(self, mission: Mission, result: str):
        with self._lock:
            if self._mission is mission:
                self._mission = None
        pose = self.get_pose()
        target = mission.targets[-1] if result == 'succeeded' else mission.targets[mission.index]
        final_error = math.dist((pose['x'], pose['y']), (target['x'], target['y'])) \
            if pose['available'] else None
        clearance = mission.min_range - BODY_RADIUS_M if math.isfinite(mission.min_range) else None
        texts = {'succeeded': 'Chegou ao destino', 'aborted': 'Não conseguiu chegar ao destino',
                 'canceled': 'Navegação cancelada', 'rejected': 'Destino recusado pelo Nav2'}
        self._emit_goal(mission, result, texts.get(result, result))
        self._emit('path_update', {'points': []})
        if mission.log_id is not None:
            try:
                db.log_finish(mission.log_id, result, time.monotonic() - mission.started,
                              mission.distance, mission.recoveries, clearance,
                              pose if pose['available'] else None, final_error)
                self._emit('history_update', {})
            except Exception:
                log.exception('falha ao fechar o registro da navegação')


# ── Ciclo de vida ────────────────────────────────────────────────────────────

_node: RosBridge | None = None


def start(emit: Callable[[str, dict], None]) -> RosBridge:
    global _node
    # Sem os tratadores de sinal do rclpy: quem trata Ctrl+C/SIGTERM é o app
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    _node = RosBridge(emit)
    threading.Thread(target=rclpy.spin, args=(_node,), daemon=True).start()
    return _node


def node() -> RosBridge | None:
    return _node
