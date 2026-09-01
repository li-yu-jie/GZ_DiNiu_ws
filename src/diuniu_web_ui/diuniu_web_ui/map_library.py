# =============================================================================
# map_library.py — 多地图库
#
# 布局（MAP_DIR = <ws>/src/diuniu_nav/maps）：
#   map.pgm / map.yaml / keepout_mask.* / waypoints.json   ← 固定入口（launch 写死，不动）
#   library/index.json                                     ← {maps: [...], active: "<id>"}
#   library/<id>/map.pgm|map.yaml|keepout_mask.*|waypoints.json
#
# 切换地图 = 把库内条目文件复制到固定入口 + load_map 热加载；
# 导航未运行时仅落盘，下次启动导航自动生效（_load_map_service 已处理该语义）。
# =============================================================================
import json
import os
import re
import shutil
import threading
import time
import uuid

from .map_store import (KEEPOUT_PGM, KEEPOUT_YAML, MAP_DIR, MAP_PGM, MAP_YAML,
                        WAYPOINTS_JSON, _load_map_service)

LIB_DIR = os.path.join(MAP_DIR, 'library')
INDEX_JSON = os.path.join(LIB_DIR, 'index.json')
_LIB_DIR_REAL = os.path.realpath(LIB_DIR)

# map_id 白名单：拒绝 / 和 ..，杜绝路径遍历（delete 会 rmtree，必须严格）
_MAP_ID_RE = re.compile(r'^[0-9a-zA-Z_-]{1,64}$')

# 模块级锁：main.py 的全局实例和建图保存流水线里的临时实例会并发读写
# index.json（read-modify-write），实例级锁互不互斥，曾可能写坏索引
_lib_lock = threading.Lock()

# 条目内文件名 → 固定入口路径
ENTRY_FILES = {
    'map.pgm': MAP_PGM,
    'map.yaml': MAP_YAML,
    'keepout_mask.pgm': KEEPOUT_PGM,
    'keepout_mask.yaml': KEEPOUT_YAML,
    'waypoints.json': WAYPOINTS_JSON,
}


class MapLibrary:
    """地图库：list / create_from_current / sync_active / activate / rename / delete。

    所有方法用模块级 _lib_lock，跨实例互斥；不要换成实例锁。"""

    def __init__(self):
        os.makedirs(LIB_DIR, exist_ok=True)
        # 迁移：首次升级时把现有固定入口地图导入为"默认地图"
        with _lib_lock:
            if not os.path.exists(INDEX_JSON) and os.path.exists(MAP_PGM):
                entry = self._new_entry('默认地图')
                self._copy_entry_files(MAP_DIR, self._entry_dir(entry['id']), skip_missing=True)
                self._save_index({'maps': [entry], 'active': entry['id']})

    # ---------------- 内部 ----------------
    @staticmethod
    def _new_entry(name):
        return {'id': uuid.uuid4().hex[:8], 'name': name,
                'created_at': time.strftime('%Y-%m-%dT%H:%M:%S')}

    @staticmethod
    def _entry_dir(map_id):
        return os.path.join(LIB_DIR, map_id)

    @staticmethod
    def _entry_dir_checked(map_id):
        """校验外部传入的 map_id 并返回条目目录；非法或越出地图库目录返回 None。

        正则白名单之外再做 realpath 归属检查（防符号链接等绕过），
        所有接收外部 map_id 的入口必须走这里。"""
        if not _MAP_ID_RE.match(map_id or ''):
            return None
        path = os.path.realpath(os.path.join(LIB_DIR, map_id))
        if path == _LIB_DIR_REAL or os.path.commonpath([_LIB_DIR_REAL, path]) != _LIB_DIR_REAL:
            return None
        return path

    def _load_index(self):
        try:
            with open(INDEX_JSON) as f:
                return json.load(f)
        except Exception:
            return {'maps': [], 'active': None}

    def _save_index(self, idx):
        with open(INDEX_JSON, 'w') as f:
            json.dump(idx, f, ensure_ascii=False, indent=2)

    @staticmethod
    def _copy_entry_files(src_dir, dst_dir, skip_missing):
        """按 ENTRY_FILES 清单在 src_dir/dst_dir 之间复制（dst_dir 需已存在）。"""
        os.makedirs(dst_dir, exist_ok=True)
        for fname in ENTRY_FILES:
            src = os.path.join(src_dir, fname)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(dst_dir, fname))
            elif not skip_missing:
                raise FileNotFoundError(src)

    # ---------------- 查询 ----------------
    def list(self):
        with _lib_lock:
            return self._load_index()

    # ---------------- 建图保存接入 ----------------
    def create_from_current(self, name):
        """把当前固定入口文件存为新地图并置为 active。返回新条目。"""
        with _lib_lock:
            idx = self._load_index()
            entry = self._new_entry(name)
            self._copy_entry_files(MAP_DIR, self._entry_dir(entry['id']), skip_missing=True)
            idx['maps'].append(entry)
            idx['active'] = entry['id']
            self._save_index(idx)
            return entry

    def sync_active_from_current(self):
        """覆盖式保存后，把固定入口文件同步回 active 库条目。"""
        with _lib_lock:
            idx = self._load_index()
            entry = next((m for m in idx['maps'] if m['id'] == idx.get('active')), None)
            if not entry:
                return
            self._copy_entry_files(MAP_DIR, self._entry_dir(entry['id']), skip_missing=True)

    # ---------------- 切换 / 管理 ----------------
    def activate(self, map_id):
        """切换激活地图：库文件 → 固定入口 + 热加载。返回 (ok, msg)。"""
        edir = self._entry_dir_checked(map_id)
        if edir is None:
            return False, '非法地图 ID'
        with _lib_lock:
            idx = self._load_index()
            entry = next((m for m in idx['maps'] if m['id'] == map_id), None)
            if not entry:
                return False, '地图不存在'
            if not os.path.exists(os.path.join(edir, 'map.pgm')):
                return False, '该地图的文件缺失（map.pgm）'
            # 条目里有的文件复制到固定入口；没有的删掉旧固定文件，避免串图
            for fname, fixed in ENTRY_FILES.items():
                src = os.path.join(edir, fname)
                if os.path.exists(src):
                    shutil.copy2(src, fixed)
                elif os.path.exists(fixed):
                    os.remove(fixed)
            idx['active'] = map_id
            self._save_index(idx)
        ok, msg = _load_map_service('map_server', MAP_YAML)
        if os.path.exists(KEEPOUT_PGM):
            _load_map_service('filter_mask_server', KEEPOUT_YAML)
        return ok, f'已切换到「{entry["name"]}」：{msg}'

    def rename(self, map_id, name):
        name = (name or '').strip()
        if not name:
            return False, '名称不能为空'
        if self._entry_dir_checked(map_id) is None:
            return False, '非法地图 ID'
        with _lib_lock:
            idx = self._load_index()
            entry = next((m for m in idx['maps'] if m['id'] == map_id), None)
            if not entry:
                return False, '地图不存在'
            entry['name'] = name
            self._save_index(idx)
            return True, '已重命名'

    def delete(self, map_id):
        edir = self._entry_dir_checked(map_id)
        if edir is None:
            return False, '非法地图 ID'
        with _lib_lock:
            idx = self._load_index()
            entry = next((m for m in idx['maps'] if m['id'] == map_id), None)
            if not entry:
                return False, '地图不存在'
            if idx.get('active') == map_id:
                return False, '当前使用的地图不能删除，请先切换到其他地图'
            idx['maps'] = [m for m in idx['maps'] if m['id'] != map_id]
            self._save_index(idx)
            shutil.rmtree(edir, ignore_errors=True)
            return True, '已删除'
