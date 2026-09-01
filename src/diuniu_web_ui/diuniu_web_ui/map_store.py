# =============================================================================
# map_store.py — 地图文件 / 禁区 mask / 导航点 的读写与热加载
#
# pgm 读写用 PIL（'L' 灰度即 ROS trinary pgm：0=占用 205=未知 254=空闲）。
# 热加载通过 ros2 service call <node>/load_map (nav2_msgs/srv/LoadMap)。
# =============================================================================
import io
import json
import os
import subprocess
import threading

import yaml
from PIL import Image

WS = os.environ.get('DIUNIU_WS', os.path.expanduser('~/GZ_DiNiu_ws'))
MAP_DIR = os.path.join(WS, 'src/diuniu_nav/maps')
MAP_PGM = os.path.join(MAP_DIR, 'map.pgm')
MAP_YAML = os.path.join(MAP_DIR, 'map.yaml')
KEEPOUT_PGM = os.path.join(MAP_DIR, 'keepout_mask.pgm')
KEEPOUT_YAML = os.path.join(MAP_DIR, 'keepout_mask.yaml')
WAYPOINTS_JSON = os.path.join(MAP_DIR, 'waypoints.json')

SHELL_ENV = f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash'

_file_lock = threading.Lock()


def read_map_yaml():
    with open(MAP_YAML) as f:
        return yaml.safe_load(f)


def pgm_to_png_bytes(pgm_path):
    img = Image.open(pgm_path).convert('L')
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue(), img.size


def _load_map_service(server_name, yaml_path, timeout=30):
    """调用 nav2 map_server 的 load_map 服务热加载地图。
    返回 (ok, msg)：服务不可用（导航未启动）也算 ok——文件已落盘，下次启动生效。"""
    cmd = (f'{SHELL_ENV} && ros2 service call /{server_name}/load_map '
           f'nav2_msgs/srv/LoadMap "{{map_url: {yaml_path}}}"')
    try:
        r = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=timeout)
        out = (r.stdout or '') + (r.stderr or '')
        ok = (r.returncode == 0) and ('result=0' in out or 'RESULT_SUCCESS' in out or 'result: 0' in out)
        if ok:
            return True, '保存成功，地图已热加载生效'
        if r.returncode != 0:
            return True, '已保存到文件（导航模式未运行，启动导航后自动生效）'
        return False, f'load_map 返回异常: {out[-300:]}'
    except subprocess.TimeoutExpired:
        return True, '已保存到文件（热加载超时，启动导航后自动生效）'
    except Exception as e:
        return False, str(e)


class MapStore:
    """地图 pgm 与禁区 mask 的读取/保存 + 热加载。"""

    # ---------------- 地图修图 ----------------
    def get_map_image(self):
        """返回 (png_bytes, (w,h), map_yaml_dict)。"""
        with _file_lock:
            png, size = pgm_to_png_bytes(MAP_PGM)
            return png, size, read_map_yaml()

    def save_map_image(self, png_bytes):
        """保存前端修图结果（PNG 灰度）到 map.pgm，并热加载。返回 (ok, msg)。"""
        img = Image.open(io.BytesIO(png_bytes)).convert('L')
        with _file_lock:
            if not os.path.exists(MAP_PGM):
                return False, '当前地图文件 (map.pgm) 不存在'
            cur_size = Image.open(MAP_PGM).size
            if img.size != cur_size:
                return False, f'图片尺寸 {img.size} 与当前地图 {cur_size} 不一致'
            img.save(MAP_PGM)  # PIL 'L' → P5 pgm
        return _load_map_service('map_server', MAP_YAML)

    # ---------------- 禁区 mask ----------------
    def get_keepout_image(self):
        """返回 (png_bytes, (w,h))；mask 不存在时按地图尺寸生成全白。"""
        with _file_lock:
            if not os.path.exists(KEEPOUT_PGM):
                self._write_keepout_locked(Image.new('L', Image.open(MAP_PGM).size, 254))
            png, size = pgm_to_png_bytes(KEEPOUT_PGM)
            return png, size

    def save_keepout_image(self, png_bytes):
        """保存禁区 mask（黑=禁区 白=通行），同步 yaml 并热加载。返回 (ok, msg)。"""
        img = Image.open(io.BytesIO(png_bytes)).convert('L')
        with _file_lock:
            cur_size = Image.open(MAP_PGM).size
            if img.size != cur_size:
                return False, f'mask 尺寸 {img.size} 与地图 {cur_size} 不一致'
            self._write_keepout_locked(img)
        return _load_map_service('filter_mask_server', KEEPOUT_YAML)

    @staticmethod
    def _write_keepout_locked(img):
        img.save(KEEPOUT_PGM)
        y = read_map_yaml()
        out = {'image': 'keepout_mask.pgm', 'mode': 'trinary',
               'resolution': y['resolution'], 'origin': y['origin'],
               'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
        with open(KEEPOUT_YAML, 'w') as f:
            yaml.dump(out, f, default_flow_style=False)


class WaypointStore:
    """导航点持久化：maps/waypoints.json，元素 {id, name, x, y, yaw}（map 坐标系）。

    锁是模块级的：建图保存流水线会另建实例清空导航点，实例锁互不互斥。"""

    _file_lock = threading.Lock()   # 类属性 = 全实例共享

    def __init__(self):
        os.makedirs(MAP_DIR, exist_ok=True)
        if not os.path.exists(WAYPOINTS_JSON):
            self._save([])

    def _load(self):
        try:
            with open(WAYPOINTS_JSON) as f:
                return json.load(f)
        except Exception:
            return []

    def _save(self, wps):
        with open(WAYPOINTS_JSON, 'w') as f:
            json.dump(wps, f, ensure_ascii=False, indent=2)

    def list(self):
        with self._file_lock:
            return self._load()

    def add(self, name, x, y, yaw):
        with self._file_lock:
            wps = self._load()
            new_id = (max([w['id'] for w in wps]) + 1) if wps else 1
            wp = {'id': new_id, 'name': name or f'点{new_id}',
                  'x': float(x), 'y': float(y), 'yaw': float(yaw)}
            wps.append(wp)
            self._save(wps)
            return wp

    def update(self, wp_id, **fields):
        with self._file_lock:
            wps = self._load()
            for w in wps:
                if w['id'] == wp_id:
                    for k in ('name', 'x', 'y', 'yaw'):
                        if k in fields and fields[k] is not None:
                            w[k] = fields[k]
                    self._save(wps)
                    return w
            return None

    def delete(self, wp_id):
        with self._file_lock:
            wps = self._load()
            new = [w for w in wps if w['id'] != wp_id]
            self._save(new)
            return len(new) != len(wps)

    def clear(self):
        """清空全部导航点（建图存为新地图时调用，新图坐标系不沿用旧点）。"""
        with self._file_lock:
            self._save([])
