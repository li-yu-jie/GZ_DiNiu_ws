# =============================================================================
# process_manager.py — 建图/导航模式进程组管理 + 建图保存流水线
#
# 所有 ros2 命令都通过 bash 先 source ROS 与工作区环境再执行。
# 进程组以 setsid 启动，停止时向整个进程组发 SIGINT（FAST-LIO 借此落盘 PCD）。
# =============================================================================
import os
import signal
import subprocess
import threading
import time

WS = os.environ.get('DIUNIU_WS', os.path.expanduser('~/GZ_DiNiu_ws'))
PCD_FILE = os.path.join(WS, 'src/FAST_LIO/PCD/scans.pcd')
PCD2PGM_PARAMS = os.path.join(WS, 'src/pcd2pgm/config/pcd2pgm.yaml')
MAP_DIR = os.path.join(WS, 'src/diuniu_nav/maps')

SHELL_ENV = f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash'

MODE_LAUNCH = {
    'mapping': 'ros2 launch diuniu_nav diuniu_mapping.launch.py',
    'navigation': 'ros2 launch diuniu_nav diuniu_nav_all.launch.py',
}


def bash_cmd(cmd):
    return ['bash', '-c', f'{SHELL_ENV} && {cmd}']


class ProcessManager:
    """管理建图/导航两种模式的后台进程组，以及建图保存流水线。"""

    def __init__(self, logger=print):
        self.log = logger
        self._proc = None          # 当前模式进程 (subprocess.Popen)
        self._mode = 'stopped'     # stopped / mapping / navigation
        self._lock = threading.Lock()
        # 保存流水线状态（前端轮询）
        self.save_status = {
            'running': False, 'done': False, 'error': '',
            'step': '', 'log': []
        }

    # ---------------- 模式管理 ----------------
    def start_mode(self, mode):
        if mode not in MODE_LAUNCH:
            return False, f'未知模式: {mode}'
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False, f'已有模式 {self._mode} 在运行，请先停止'
            try:
                self._proc = subprocess.Popen(
                    bash_cmd(MODE_LAUNCH[mode]),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid)
                self._mode = mode
                self.log(f'[ProcessManager] 启动模式 {mode}, pid={self._proc.pid}')
                return True, f'{mode} 已启动'
            except Exception as e:
                return False, str(e)

    def stop_mode(self):
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self._proc = None
                self._mode = 'stopped'
                return True, '已停止'
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            # 最多等 20s 让 FAST-LIO 落盘 PCD / Nav2 正常退出
            for _ in range(200):
                if self._proc.poll() is not None:
                    break
                time.sleep(0.1)
            if self._proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self._proc = None
            self._mode = 'stopped'
            return True, '已停止'

    def status(self):
        running = self._proc is not None and self._proc.poll() is not None
        return {'mode': self._mode if running else 'stopped',
                'running': running,
                'pid': self._proc.pid if running else None}

    # ---------------- 建图保存流水线 ----------------
    def start_mapping_save(self):
        if self.save_status['running']:
            return False, '保存流水线正在运行'
        if self._mode != 'mapping' or self._proc is None or self._proc.poll() is not None:
            return False, '当前不在建图模式，无法保存'
        t = threading.Thread(target=self._mapping_save_pipeline, daemon=True)
        t.start()
        return True, '保存流水线已启动'

    def _save_log(self, msg):
        self.save_status['log'].append(f'[{time.strftime("%H:%M:%S")}] {msg}')
        self.log(f'[MappingSave] {msg}')

    def _run(self, cmd, timeout=120):
        """同步执行 ros2 命令，返回 (returncode, stdout+stderr)。"""
        r = subprocess.run(bash_cmd(cmd), capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or '') + (r.stderr or '')

    def _mapping_save_pipeline(self):
        s = self.save_status
        s.update({'running': True, 'done': False, 'error': '', 'step': '停止建图节点', 'log': []})
        tmp_dir = os.path.join('/tmp', 'diuniu_map_save')
        os.makedirs(tmp_dir, exist_ok=True)
        pcd2pgm_proc = None
        try:
            # 1. SIGINT 停建图进程组 → FAST-LIO 落盘 scans.pcd
            self._save_log('停止建图节点（FAST-LIO 正在保存 PCD...）')
            old_mtime = os.path.getmtime(PCD_FILE) if os.path.exists(PCD_FILE) else 0
            self.stop_mode()
            if not os.path.exists(PCD_FILE):
                raise RuntimeError(f'PCD 文件不存在: {PCD_FILE}')
            new_mtime = os.path.getmtime(PCD_FILE)
            self._save_log(f'PCD 已就绪: {PCD_FILE} ({"新落盘" if new_mtime > old_mtime else "沿用旧文件"})')

            # 2. 启动 pcd2pgm：PCD → /map (OccupancyGrid, transient_local)
            s['step'] = 'PCD 转栅格图'
            self._save_log('启动 pcd2pgm 转换点云...')
            pcd2pgm_proc = subprocess.Popen(
                bash_cmd(f'ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file {PCD2PGM_PARAMS}'),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid)
            time.sleep(4)  # 等待加载点云并发布 /map

            # 3. map_saver_cli 订阅 /map 落盘 pgm/yaml
            self._save_log('保存栅格地图 (map_saver_cli)...')
            tmp_map = os.path.join(tmp_dir, 'map')
            rc, out = self._run(
                f'ros2 run nav2_map_server map_saver_cli -t map -f {tmp_map} '
                f'--ros-args -p map_subscribe_transient_local:=true', timeout=60)
            if rc != 0 or not os.path.exists(tmp_map + '.pgm'):
                raise RuntimeError(f'map_saver_cli 失败: {out[-500:]}')
            self._save_log('栅格地图保存成功')

            # 4. 覆盖导航地图 + 重置禁区 mask（新图尺寸/原点可能变化）
            s['step'] = '更新导航地图'
            self._save_log(f'覆盖 {MAP_DIR}/map.pgm / map.yaml')
            subprocess.run(['cp', tmp_map + '.pgm', os.path.join(MAP_DIR, 'map.pgm')], check=True)
            subprocess.run(['cp', tmp_map + '.yaml', os.path.join(MAP_DIR, 'map.yaml')], check=True)
            # map_saver_cli 生成的 yaml 里 image 是绝对/相对路径，统一改回 map.pgm
            self._fix_map_yaml(os.path.join(MAP_DIR, 'map.yaml'))
            self._reset_keepout_mask()
            self._save_log('禁区 mask 已重置为全空')

            # 5. 重启导航模式加载新图
            s['step'] = '重启导航模式'
            self._save_log('重启导航模式加载新地图...')
            ok, msg = self.start_mode('navigation')
            if not ok:
                raise RuntimeError(f'导航模式启动失败: {msg}')

            s.update({'running': False, 'done': True, 'step': '完成'})
            self._save_log('✅ 建图保存完成，已切换回导航模式')
        except Exception as e:
            s.update({'running': False, 'done': False, 'error': str(e), 'step': '失败'})
            self._save_log(f'❌ 保存失败: {e}')
        finally:
            if pcd2pgm_proc is not None and pcd2pgm_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(pcd2pgm_proc.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass

    @staticmethod
    def _fix_map_yaml(yaml_path):
        """map_saver_cli 生成的 yaml 中 image 字段可能带 tmp 路径，统一改回 map.pgm。"""
        try:
            import yaml
            with open(yaml_path) as f:
                y = yaml.safe_load(f)
            y['image'] = 'map.pgm'
            y.setdefault('mode', 'trinary')
            with open(yaml_path, 'w') as f:
                yaml.dump(y, f, default_flow_style=False)
        except Exception:
            pass

    @staticmethod
    def _reset_keepout_mask():
        """按新 map.pgm 尺寸/原点生成全白禁区 mask（无禁区）。"""
        try:
            from PIL import Image
            import yaml
            map_pgm = os.path.join(MAP_DIR, 'map.pgm')
            map_yaml = os.path.join(MAP_DIR, 'map.yaml')
            size = Image.open(map_pgm).size
            Image.new('L', size, 254).save(os.path.join(MAP_DIR, 'keepout_mask.pgm'))
            with open(map_yaml) as f:
                y = yaml.safe_load(f)
            out = {'image': 'keepout_mask.pgm', 'mode': 'trinary',
                   'resolution': y['resolution'], 'origin': y['origin'],
                   'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25}
            with open(os.path.join(MAP_DIR, 'keepout_mask.yaml'), 'w') as f:
                yaml.dump(out, f, default_flow_style=False)
        except Exception:
            pass
