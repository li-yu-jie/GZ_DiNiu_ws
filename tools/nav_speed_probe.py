#!/usr/bin/env python3
# nav_speed_probe.py — 导航速度链实时探针
# 同步采样：RPP 输出 /cmd_vel_nav → 最终 /cmd_vel → 实际里程计 /odom
# 以及全局路径长度 /plan 与导航反馈 distance_remaining
# 用于区分：RPP 主动慢速 / 底盘执行不跟踪 / 终点对中悬停 / DUBIN 绕大圈
import time
import importlib
import rclpy
from rclpy.node import Node

TOPICS = {
    'odom': '/odom',
    'rpp':  '/cmd_vel_nav',
    'cmd':  '/cmd_vel',
    'plan': '/plan',
    'fb':   '/navigate_to_pose/_action/feedback',
}


def load_class(type_str):
    # "nav_msgs/msg/Odometry" -> nav_msgs.msg.Odometry
    pkg, kind, name = type_str.split('/')
    return getattr(importlib.import_module(f'{pkg}.{kind}'), name)


def twist_of(msg):
    t = getattr(msg, 'twist', msg)                 # Twist / TwistStamped
    t = getattr(t, 'twist', t)                     # Odometry(.twist.twist)
    return t.linear.x, t.angular.z


class Probe(Node):
    def __init__(self):
        super().__init__('nav_speed_probe')
        self.last = {}
        self.t0 = time.time()
        types = dict(self.get_topic_names_and_types())
        for key, topic in TOPICS.items():
            tlist = types.get(topic)
            if not tlist:
                print(f'## 话题不存在: {topic}', flush=True)
                continue
            try:
                cls = load_class(tlist[0])
            except Exception as e:
                print(f'## {topic} 类型加载失败 {tlist[0]}: {e}', flush=True)
                continue
            self.create_subscription(cls, topic,
                                     lambda m, k=key: self.cb(k, m), 10)
        self.create_timer(0.25, self.tick)

    def cb(self, key, msg):
        if key in ('odom', 'rpp', 'cmd'):
            self.last[key] = twist_of(msg)
        elif key == 'plan':
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            L = sum(((b[0]-a[0])**2 + (b[1]-a[1])**2) ** 0.5
                    for a, b in zip(pts, pts[1:]))
            self.last['plan'] = (len(pts), L)
        elif key == 'fb':
            fb = msg.feedback
            self.last['fb'] = (fb.distance_remaining,)

    def tick(self):
        o = self.last.get('odom', (0.0, 0.0))
        n = self.last.get('rpp', (0.0, 0.0))
        c = self.last.get('cmd', (0.0, 0.0))
        pl = self.last.get('plan')
        fb = self.last.get('fb')
        line = (f'{time.time()-self.t0:6.2f}s '
                f'RPP v={n[0]:+.3f} w={n[1]:+.3f} | '
                f'cmd v={c[0]:+.3f} w={c[1]:+.3f} | '
                f'实际 v={o[0]:+.3f} w={o[1]:+.3f}')
        if pl:
            line += f' | 路径 {pl[0]}点 {pl[1]:.2f}m'
        if fb:
            line += f' | 剩余 {fb[0]:.2f}m'
        print(line, flush=True)


rclpy.init()
rclpy.spin(Probe())
