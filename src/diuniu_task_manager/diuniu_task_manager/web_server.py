#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# FMS Web 调度终端（Flask）。
# 绑定地址：默认仅监听本机回环 127.0.0.1；如需局域网/远程访问（手机等），
# 由部署方显式设置环境变量 FMS_BIND_ADDR（如 0.0.0.0）放开。端口固定 5000。

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, request, jsonify
import threading
import os
import math

app = Flask(__name__, template_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'templates'))
ros_node = None

def is_valid_endpoint(value):
    """站点名（短字母数字串）或 'x,y[,yaw]' 坐标（有限数值，|x|,|y| < 100）。"""
    if not isinstance(value, str) or not value:
        return False
    if value.isalnum() and len(value) <= 8:
        return True
    try:
        parts = [float(v) for v in value.split(',')]
    except ValueError:
        return False
    if len(parts) not in (2, 3):
        return False
    if not all(math.isfinite(v) for v in parts):
        return False
    return abs(parts[0]) < 100 and abs(parts[1]) < 100

def is_coord_endpoint(value):
    return isinstance(value, str) and ',' in value

class WebServerNode(Node):
    def __init__(self):
        super().__init__('fms_web_server')
        self.pub = self.create_publisher(String, '/dispatch_task', 10)

    def send_task(self, pickup, dropoff):
        msg = String()
        if is_coord_endpoint(pickup):
            p = [v.strip() for v in pickup.split(',')]
            d = [v.strip() for v in dropoff.split(',')]
            if len(p) == 2:
                p.append('0.0')
            if len(d) == 2:
                d.append('0.0')
            msg.data = f"PICKUP_COORD: {','.join(p)}, DROPOFF_COORD: {','.join(d)}"
        else:
            msg.data = f"PICKUP: {pickup}, DROPOFF: {dropoff}"
        self.pub.publish(msg)
        self.get_logger().info(f"Published task: {msg.data}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/dispatch', methods=['POST'])
def dispatch():
    data = request.get_json(silent=True)
    if data is None:
        return jsonify({"status": "error", "message": "请求体必须是合法 JSON"}), 400
    pickup = data.get('pickup')
    dropoff = data.get('dropoff')
    if not pickup or not dropoff:
        return jsonify({"status": "error", "message": "缺少 pickup/dropoff 参数"}), 400
    if not is_valid_endpoint(pickup) or not is_valid_endpoint(dropoff):
        return jsonify({"status": "error", "message": "站点/坐标参数非法（坐标须为有限数值且 |x|,|y| < 100）"}), 400
    if is_coord_endpoint(pickup) != is_coord_endpoint(dropoff):
        return jsonify({"status": "error", "message": "取货点与卸货点须同为站点名或同为坐标"}), 400
    if not ros_node:
        return jsonify({"status": "error", "message": "ROS 节点未就绪"}), 503
    ros_node.send_task(pickup, dropoff)
    return jsonify({"status": "success", "message": f"任务已下发: 从 {pickup} 到 {dropoff}"})

def ros2_thread():
    rclpy.init()
    global ros_node
    ros_node = WebServerNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    # Start ROS 2 in a background thread
    t = threading.Thread(target=ros2_thread)
    t.daemon = True
    t.start()

    # Run Flask server (blocking)
    # 默认仅绑定 127.0.0.1；远程访问由部署方通过环境变量 FMS_BIND_ADDR 显式放开
    bind_addr = os.environ.get('FMS_BIND_ADDR', '127.0.0.1')
    app.run(host=bind_addr, port=5000, debug=False, use_reloader=False)

if __name__ == '__main__':
    main()
