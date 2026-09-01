# =============================================================================
# process_manager.py — 建图/导航模式进程组管理 + 建图保存流水线
#
# 所有 ros2 命令都通过 bash 先 source ROS 与工作区环境再执行。
# 进程组以 setsid 启动，停止时向整个进程组发 SIGINT（FAST-LIO 借此落盘 PCD）。
# =============================================================================
import os
import shutil
import signal
import subprocess
import tempfile
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
MODE_LOG = os.path.expanduser('~/diuniu_mode.log')
MODE_LOG_MAX_BYTES = 5 * 1024 * 1024   # 超过则轮转成 .log.1（只留一份旧档）


def _rotate_mode_log():
    """模式日志超过上限时轮转成 .log.1，防止无限增长。"""
    try:
        if os.path.getsize(MODE_LOG) > MODE_LOG_MAX_BYTES:
            os.replace(MODE_LOG, MODE_LOG + '.1')
    except OSError:
        pass


def bash_cmd(cmd):
    return ['bash', '-c', f'{SHELL_ENV} && {cmd}']


class ProcessManager:
    """管理建图/导航两种模式的后台进程组，以及建图保存流水线。"""

    def __init__(self, logger=print):
        self.log = logger
        self._proc = None          # 当前模式进程 (subprocess.Popen)
        self._logf = None          # start_mode 打开的模式日志句柄（stop_mode 关闭）
        self._mode = 'stopped'     # stopped / mapping / navigation
        self._lock = threading.Lock()
        # 保存流水线状态（前端轮询）；读写都须持 _save_lock
        self._save_lock = threading.Lock()
        self.save_status = {
            'running': False, 'done': False, 'error': '',
            'step': '', 'log': []
        }

    def _close_logf(self):
        """关闭模式日志句柄。调用方须已持有 self._lock。"""
        f, self._logf = self._logf, None
        if f is not None:
            try:
                f.close()
            except OSError:
                pass

    # ---------------- 模式管理 ----------------
    def start_mode(self, mode):
        if mode not in MODE_LAUNCH:
            return False, f'未知模式: {mode}'
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return False, f'已有模式 {self._mode} 在运行，请先停止'
            # 上一模式若自行退出（未走 stop_mode），其日志句柄可能还开着，先关掉
            self._close_logf()
            logf = None
            try:
                # 模式栈日志落盘（原先 /dev/null，节点崩溃无从排查）
                _rotate_mode_log()
                logf = open(MODE_LOG, 'ab', buffering=0)
                logf.write(f'\n===== [{mode}] 启动于 {time.strftime("%F %T")} =====\n'.encode())
                self._proc = subprocess.Popen(
                    bash_cmd(MODE_LAUNCH[mode]),
                    stdout=logf,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid)
                self._logf = logf
                self._mode = mode
                self.log(f'[ProcessManager] 启动模式 {mode}, pid={self._proc.pid}, 日志={MODE_LOG}')
                return True, f'{mode} 已启动'
            except Exception as e:
                if logf is not None:
                    try:
                        logf.close()
                    except OSError:
                        pass
                return False, str(e)

    def stop_mode(self):
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                self._proc = None
                self._mode = 'stopped'
                self._close_logf()
                return True, '已停止'
            proc = self._proc
        # 等待 FAST-LIO 落盘 / Nav2 退出最长 20s —— 必须持锁外进行，
        # 否则 status() 轮询在停止期间整体卡死
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        for _ in range(200):
            if proc.poll() is not None:
                break
            time.sleep(0.1)
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        with self._lock:
            # 等待期间若有新 start_mode 抢入（会失败于"已有模式在运行"），
            # 这里只清自己那个 proc 的登记
            if self._proc is proc:
                self._proc = None
                self._mode = 'stopped'
                self._close_logf()
        return True, '已停止'

    def status(self):
        with self._lock:
            running = self._proc is not None and self._proc.poll() is None
            return {'mode': self._mode if running else 'stopped',
                    'running': running,
                    'pid': self._proc.pid if running else None}


    # ---------------- 建图保存流水线 ----------------
    def get_save_status(self):
        """save_status 的快照拷贝（含 log 列表），供 HTTP 层安全读取。"""
        with self._save_lock:
            snap = dict(self.save_status)
            snap['log'] = list(self.save_status['log'])
            return snap

    def _save_update(self, **kw):
        with self._save_lock:
            self.save_status.update(kw)

    def start_mapping_save(self, map_name=None):
        """返回 (ok, msg, conflict)；conflict=True 表示已有保存任务进行中（HTTP 409）。"""
        with self._save_lock:
            if self.save_status['running']:
                return False, '保存流水线正在运行', True
            # 先占位再检查模式，防止并发请求双双通过 running 检查
            self.save_status.update({'running': True, 'done': False, 'error': '',
                                     'step': '停止建图节点', 'log': []})
        with self._lock:
            mapping_active = (self._mode == 'mapping' and self._proc is not None
                              and self._proc.poll() is None)
        if not mapping_active:
            self._save_update(running=False)
            return False, '当前不在建图模式，无法保存', False
        t = threading.Thread(target=self._mapping_save_pipeline, args=(map_name,), daemon=True)
        t.start()
        return True, '保存流水线已启动', False

    def _save_log(self, msg):
        with self._save_lock:
            self.save_status['log'].append(f'[{time.strftime("%H:%M:%S")}] {msg}')
        self.log(f'[MappingSave] {msg}')

    def _run(self, cmd, timeout=120):
        """同步执行 ros2 命令，返回 (returncode, stdout+stderr)。"""
        r = subprocess.run(bash_cmd(cmd), capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or '') + (r.stderr or '')

    def _mapping_save_pipeline(self, map_name=None):
        # 临时目录用 mkdtemp：固定路径会被并发/多用户场景互相踩踏
        tmp_dir = tempfile.mkdtemp(prefix='diuniu_map_save_')
        pcd2pgm_proc = None
        try:
            # 1. SIGINT 停建图进程组 → FAST-LIO 落盘 scans.pcd
            self._save_log('停止建图节点（FAST-LIO 正在保存 PCD...）')
            old_mtime = os.path.getmtime(PCD_FILE) if os.path.exists(PCD_FILE) else 0
            self.stop_mode()
            if not os.path.exists(PCD_FILE):
                raise RuntimeError(f'PCD 文件不存在: {PCD_FILE}')
            new_mtime = os.path.getmtime(PCD_FILE)
            if new_mtime <= old_mtime:
                # FAST-LIO 没落盘成功还继续走，会把旧 PCD 存成"新地图"——
                # 用户无从察觉，必须中止
                raise RuntimeError(
                    'FAST-LIO 未落盘新 PCD（mtime 未更新），保存已中止。'
                    '请检查建图日志确认 FAST-LIO 是否正常收到 SIGINT')
            self._save_log(f'PCD 已就绪: {PCD_FILE}（新落盘）')

            # 2. 启动 pcd2pgm：PCD → /map (OccupancyGrid, transient_local)
            self._save_update(step='PCD 转栅格图')
            self._save_log('启动 pcd2pgm 转换点云...')
            pcd2pgm_proc = subprocess.Popen(
                bash_cmd(f'ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file {PCD2PGM_PARAMS}'),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid)
            self._wait_map_topic(pcd2pgm_proc)

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
            self._save_update(step='更新导航地图')
            self._save_log(f'覆盖 {MAP_DIR}/map.pgm / map.yaml')
            subprocess.run(['cp', tmp_map + '.pgm', os.path.join(MAP_DIR, 'map.pgm')], check=True)
            subprocess.run(['cp', tmp_map + '.yaml', os.path.join(MAP_DIR, 'map.yaml')], check=True)
            # map_saver_cli 生成的 yaml 里 image 是绝对/相对路径，统一改回 map.pgm
            self._fix_map_yaml(os.path.join(MAP_DIR, 'map.yaml'))
            self._reset_keepout_mask()
            self._save_log('禁区 mask 已重置为全空')

            # 4.5 登记到地图库：有名字 = 存为新地图并设为当前；无名字 = 同步回当前条目
            from .map_library import MapLibrary
            lib = MapLibrary()
            if map_name:
                # 新图坐标系不同，导航点不沿用旧图；走 WaypointStore 的共享锁，
                # 不直接写文件（Web 端可能同时在编辑导航点）
                from .map_store import WaypointStore
                WaypointStore().clear()
                entry = lib.create_from_current(map_name)
                self._save_log(f'已存为新地图「{entry["name"]}」并设为当前地图（导航点已清空）')
            else:
                lib.sync_active_from_current()
                self._save_log('已同步到地图库当前条目')

            # 5. 重启导航模式加载新图
            self._save_update(step='重启导航模式')
            self._save_log('重启导航模式加载新地图...')
            ok, msg = self.start_mode('navigation')
            if not ok:
                raise RuntimeError(f'导航模式启动失败: {msg}')

            self._save_update(running=False, done=True, step='完成')
            self._save_log('✅ 建图保存完成，已切换回导航模式')
        except Exception as e:
            self._save_update(running=False, done=False, error=str(e), step='失败')
            self._save_log(f'❌ 保存失败: {e}')
        finally:
            if pcd2pgm_proc is not None and pcd2pgm_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(pcd2pgm_proc.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
            shutil.rmtree(tmp_dir, ignore_errors=True)

    def _wait_map_topic(self, pcd2pgm_proc, timeout=180):
        """轮询等 pcd2pgm 把 /map 发布出来（点云大可远超原先硬等的 4s）。

        超时或 pcd2pgm 中途退出都报错中止，避免 map_saver_cli 傻等空话题。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if pcd2pgm_proc.poll() is not None:
                raise RuntimeError(f'pcd2pgm 已退出（rc={pcd2pgm_proc.returncode}），转换失败')
            rc, out = self._run('ros2 topic info /map', timeout=15)
            if rc == 0 and 'Publisher count' in out and 'Publisher count: 0' not in out:
                self._save_log('/map 已有发布者，开始落盘栅格地图')
                return
            time.sleep(2)
        raise RuntimeError(f'等待 pcd2pgm 发布 /map 超时（{timeout}s）')

    @staticmethod
    def _fix_map_yaml(yaml_path):
        """map_saver_cli 生成的 yaml 中 image 字段可能带 tmp 路径，统一改回 map.pgm。

        失败必须抛出——静默吞错会让导航加载到指向 /tmp 的地图 yaml。"""
        import yaml
        with open(yaml_path) as f:
            y = yaml.safe_load(f)
        y['image'] = 'map.pgm'
        y.setdefault('mode', 'trinary')
        with open(yaml_path, 'w') as f:
            yaml.dump(y, f, default_flow_style=False)

    @staticmethod
    def _reset_keepout_mask():
        """按新 map.pgm 尺寸/原点生成全白禁区 mask（无禁区）。

        失败必须抛出——旧 mask 与新图尺寸/原点不匹配会让 Nav2 filter 错位。"""
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
