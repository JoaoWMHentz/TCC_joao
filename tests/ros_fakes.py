"""Dublês para testar os nós ROS 2 sem Nav2, TF publicada nem hardware.

Os nós são criados com rclpy de verdade, mas nunca giram (spin): os testes
chamam os callbacks e o `_tick` diretamente. O cliente de action do Nav2 é
trocado por FakeActionClient, que guarda os destinos e deixa o teste decidir
quando aceitar/recusar e com que status terminar.
"""

import math
from types import SimpleNamespace

import numpy as np


class FakeFuture:

    def __init__(self, result=None, done=False):
        self._result = result
        self._done = done
        self._callbacks = []

    def add_done_callback(self, cb):
        self._callbacks.append(cb)
        if self._done:
            cb(self)

    def set_result(self, result):
        self._result, self._done = result, True
        for cb in self._callbacks:
            cb(self)

    def result(self):
        return self._result

    def done(self):
        return self._done


class FakeGoalHandle:

    def __init__(self, accepted=True):
        self.accepted = accepted
        self.canceled = False
        self._result = FakeFuture()

    def get_result_async(self):
        return self._result

    def cancel_goal_async(self):
        self.canceled = True
        return FakeFuture(done=True)

    def finish(self, status):
        self._result.set_result(SimpleNamespace(status=status))


class FakeActionClient:

    def __init__(self, ready=True):
        self.ready = ready
        self.sent = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        future = FakeFuture()
        self.sent.append(SimpleNamespace(goal=goal, future=future, feedback=feedback_callback))
        return future

    @property
    def last(self):
        return self.sent[-1].goal

    def accept(self):
        handle = FakeGoalHandle(accepted=True)
        self.sent[-1].future.set_result(handle)
        return handle

    def reject(self):
        handle = FakeGoalHandle(accepted=False)
        self.sent[-1].future.set_result(handle)
        return handle


class FakeLogger:
    """Logger mínimo (aceita os kwargs do logger do rclpy) que guarda as mensagens."""

    def __init__(self):
        self.messages = []

    def _log(self, level, msg, **_):
        self.messages.append((level, msg))

    def info(self, msg, **kw):
        self._log('info', msg, **kw)

    def warn(self, msg, **kw):
        self._log('warn', msg, **kw)

    def error(self, msg, **kw):
        self._log('error', msg, **kw)


# ── Mensagens ────────────────────────────────────────────────────────────────

def make_map(grid, res=0.05, origin=(0.0, 0.0)):
    from nav_msgs.msg import OccupancyGrid
    grid = np.asarray(grid)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = float(res)
    msg.info.height, msg.info.width = grid.shape
    msg.info.origin.position.x, msg.info.origin.position.y = map(float, origin)
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(v) for v in grid.ravel()]
    return msg


def make_scan(ranges, angle_min=-math.pi, range_max=10.0, frame='laser'):
    from sensor_msgs.msg import LaserScan
    ranges = [float(r) for r in ranges]
    msg = LaserScan()
    msg.header.frame_id = frame
    msg.angle_min = float(angle_min)
    msg.angle_increment = 2 * math.pi / len(ranges)
    msg.angle_max = msg.angle_min + msg.angle_increment * (len(ranges) - 1)
    msg.range_min, msg.range_max = 0.05, float(range_max)
    msg.ranges = ranges
    return msg


def set_tf(node, x, y, yaw=0.0, parent='map', child='base_link'):
    """Grava a transformada parent -> child direto no buffer de TF do nó."""
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y = float(x), float(y)
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    node.tf_buffer.set_transform_static(t, 'test')


def room(n=60, wall=100, free=0):
    """Sala quadrada n x n células, paredes ocupadas na borda."""
    grid = np.full((n, n), free, dtype=np.int16)
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = wall
    return grid
