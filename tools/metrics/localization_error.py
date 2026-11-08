#!/usr/bin/env python3
"""
localization_error.py — erro da localização em relação à verdade de campo.

Compara a pose estimada (TF map -> base_link, do RTAB-Map) com a pose real do
robô no simulador (/ground_truth/odom, no frame do mundo Gazebo).

O frame `map` nasce na pose do robô no início do mapeamento; informe essa pose
(no mundo) com --origin para alinhar os dois referenciais. No mundo `loja`
com os valores padrão do world.launch.py ela é (-4.0, -0.8, 0.0).

Uso (com a simulação rodando e um mapa aberto):
  python3 tools/metrics/localization_error.py --duration 120 --csv erro.csv

Saída: média, mediana, p95 e máximo do erro de posição (m) e de orientação (°).
"""

import argparse
import csv
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import tf2_ros


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Sampler(Node):

    def __init__(self, origin, period):
        super().__init__('localization_error', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.ox, self.oy, self.oyaw = origin
        self.truth = None
        self.samples = []
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 10)
        self.create_timer(period, self.sample)

    def on_truth(self, msg: Odometry):
        # Mundo -> frame map (translação e rotação da origem do mapa)
        p = msg.pose.pose.position
        dx, dy = p.x - self.ox, p.y - self.oy
        c, s = math.cos(-self.oyaw), math.sin(-self.oyaw)
        self.truth = (c * dx - s * dy, s * dx + c * dy,
                      yaw_of(msg.pose.pose.orientation) - self.oyaw)

    def sample(self):
        if self.truth is None:
            return
        try:
            tf = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        t = tf.transform.translation
        est = (t.x, t.y, yaw_of(tf.transform.rotation))
        tx, ty, tyaw = self.truth
        self.samples.append({
            't': self.get_clock().now().nanoseconds * 1e-9,
            'est_x': est[0], 'est_y': est[1], 'true_x': tx, 'true_y': ty,
            'pos_err_m': math.hypot(est[0] - tx, est[1] - ty),
            'yaw_err_deg': math.degrees(angle_diff(est[2], tyaw)),
        })


def stats(values):
    v = sorted(values)
    return {'média': sum(v) / len(v), 'mediana': v[len(v) // 2],
            'p95': v[min(len(v) - 1, int(len(v) * 0.95))], 'máx': v[-1]}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=60.0, help='segundos de amostragem')
    ap.add_argument('--period', type=float, default=0.5, help='intervalo entre amostras (s)')
    ap.add_argument('--origin', type=float, nargs=3, default=(-4.0, -0.8, 0.0),
                    metavar=('X', 'Y', 'YAW'), help='pose (mundo) do início do mapeamento')
    ap.add_argument('--csv', help='grava todas as amostras neste arquivo')
    args = ap.parse_args()

    rclpy.init()
    node = Sampler(args.origin, args.period)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.samples:
        print('Nenhuma amostra: verifique /ground_truth/odom e o TF map -> base_link.')
        return
    pos = stats([s['pos_err_m'] for s in node.samples])
    yaw = stats([s['yaw_err_deg'] for s in node.samples])
    print(f'amostras: {len(node.samples)}')
    print('erro de posição (m):    ' + '  '.join(f'{k}={v:.3f}' for k, v in pos.items()))
    print('erro de orientação (°): ' + '  '.join(f'{k}={v:.1f}' for k, v in yaw.items()))
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(node.samples[0]))
            writer.writeheader()
            writer.writerows(node.samples)
        print(f'amostras gravadas em {args.csv}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
