# =============================================================================
# main.py — FastAPI 后端入口（含 JWT/RBAC 鉴权）
#
# 职责：托管 Vue 前端静态产物 + 提供 REST API（鉴权 / 模式管理 / 建图保存 /
#       修图 / 禁区 / 航点 / 系统状态）。ROS 话题与 Action 由前端经 rosbridge 直连。
#
# 权限矩阵（后端强制）：
#   读类接口（status/map image/waypoints GET/keepout image）  viewer+
#   导航模式启停 /api/mode/navigation|stop                      operator+
#   建图模式/保存、修图、禁区、航点写、账号管理                 admin
#
# 启动：ros2 run diuniu_web_ui web_server  （或 launch/web_ui.launch.py 一并起 rosbridge）
# 访问：http://<工控机IP>:8000
#
# 安全模型：rosbridge 只监听 127.0.0.1，外部唯一的 ROS 入口是本站
# /ws/rosbridge 鉴权代理（JWT 校验通过才转发），否则 REST 的 RBAC 会被
# 直连 9090 端口旁路。HTTPS 可选：设置 DIUNIU_TLS_CERT / DIUNIU_TLS_KEY 即启用。
# 注意：同一 DDS 域内的 ROS2 节点本身无鉴权（SROS2 才有），本措施只守 Web 入口。
# =============================================================================
import asyncio
import os

import uvicorn
try:
    import websockets
except ImportError:  # 容器/新机未 pip install websockets 时用包内 vendored 副本
    from ._vendor import websockets
from fastapi import Depends, FastAPI, HTTPException, Request, WebSocket
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel

from . import db
from .auth import decode_token, get_current_user, require_role
from .map_library import MapLibrary
from .map_store import MapStore, WaypointStore
from .process_manager import ProcessManager
from .routes_auth import router as auth_router
from .routes_users import router as users_router
from .sysinfo import get_sysinfo


# ---------------- 静态目录定位（install 后优先 share 目录） ----------------
def _find_web_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('diuniu_web_ui'), 'web')
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')


WEB_DIR = _find_web_dir()

db.init_db()

# API 文档默认关闭（避免暴露接口结构）；设 DIUNIU_API_DOCS=true 才开启
_api_docs = os.environ.get('DIUNIU_API_DOCS', '').lower() == 'true'
app = FastAPI(title='地牛叉车 Web 控制端',
              docs_url='/docs' if _api_docs else None,
              redoc_url='/redoc' if _api_docs else None,
              openapi_url='/openapi.json' if _api_docs else None)
pm = ProcessManager()
maps = MapStore()
maplib = MapLibrary()
waypoints = WaypointStore()

app.include_router(auth_router)
app.include_router(users_router)

# Depends 语义速查：
#   viewer   = 任意登录用户可读
#   operator = 操作员及以上（日常搬运：导航模式启停）
#   admin    = 管理员（建图/保存地图/修图/禁区/航点写/账号管理）
_viewer = Depends(get_current_user)
_operator = Depends(require_role('operator'))
_admin = Depends(require_role('admin'))


# ---------------- 模式管理 ----------------
@app.post('/api/mode/{mode}')
def set_mode(mode: str, user: dict = _operator):
    # 建图模式属环境数据变更，收紧到管理员
    if mode == 'mapping' and user['role'] != 'admin':
        raise HTTPException(403, '建图模式仅管理员可启动')
    if mode == 'stop':
        ok, msg = pm.stop_mode()
    else:
        ok, msg = pm.start_mode(mode)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


@app.get('/api/status')
def status(_user: dict = _viewer):
    st = pm.status()
    st['sysinfo'] = get_sysinfo()
    st['save'] = pm.get_save_status()
    return st


# ---------------- 建图保存 ----------------
class MappingSaveIn(BaseModel):
    name: str | None = None   # 非空 = 存为新地图；空 = 覆盖当前地图


@app.post('/api/mapping/save')
def mapping_save(body: MappingSaveIn | None = None, _user: dict = _admin):
    name = (body.name or '').strip() if body else ''
    ok, msg, conflict = pm.start_mapping_save(name or None)
    if not ok:
        raise HTTPException(409 if conflict else 400, msg)
    return {'ok': True, 'msg': msg}


@app.get('/api/mapping/save/status')
def mapping_save_status(_user: dict = _admin):
    return pm.get_save_status()


# ---------------- 多地图库 ----------------
@app.get('/api/maps')
def list_maps(_user: dict = _viewer):
    return maplib.list()


@app.post('/api/maps/{map_id}/activate')
def activate_map(map_id: str, _user: dict = _operator):
    ok, msg = maplib.activate(map_id)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


class MapRenameIn(BaseModel):
    name: str


@app.put('/api/maps/{map_id}')
def rename_map(map_id: str, body: MapRenameIn, _user: dict = _admin):
    ok, msg = maplib.rename(map_id, body.name)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


@app.delete('/api/maps/{map_id}')
def delete_map(map_id: str, _user: dict = _admin):
    ok, msg = maplib.delete(map_id)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


# ---------------- 地图修图 ----------------
@app.get('/api/map/image')
def get_map_image(_user: dict = _viewer):
    try:
        png, size, y = maps.get_map_image()
    except FileNotFoundError:
        raise HTTPException(404, '地图文件不存在')
    return Response(content=png, media_type='image/png',
                    headers={'X-Map-Width': str(size[0]), 'X-Map-Height': str(size[1]),
                             'X-Map-Resolution': str(y['resolution']),
                             'X-Map-Origin-X': str(y['origin'][0]),
                             'X-Map-Origin-Y': str(y['origin'][1])})


MAX_IMAGE_UPLOAD_BYTES = 64 * 1024 * 1024   # 修图/禁区 PNG 上限（PGM 展开后量级）


@app.post('/api/map/save')
async def save_map_image(request: Request, _user: dict = _admin):
    body = await request.body()
    if len(body) > MAX_IMAGE_UPLOAD_BYTES:
        raise HTTPException(413, f'图片超过大小上限 {MAX_IMAGE_UPLOAD_BYTES // 2**20}MB')
    ok, msg = maps.save_map_image(body)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg + _nav_running_warning()}


# ---------------- 禁区 mask ----------------
@app.get('/api/keepout/image')
def get_keepout_image(_user: dict = _viewer):
    png, size = maps.get_keepout_image()
    return Response(content=png, media_type='image/png',
                    headers={'X-Map-Width': str(size[0]), 'X-Map-Height': str(size[1])})


@app.post('/api/keepout/save')
async def save_keepout_image(request: Request, _user: dict = _admin):
    body = await request.body()
    if len(body) > MAX_IMAGE_UPLOAD_BYTES:
        raise HTTPException(413, f'图片超过大小上限 {MAX_IMAGE_UPLOAD_BYTES // 2**20}MB')
    ok, msg = maps.save_keepout_image(body)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg + _nav_running_warning()}


# ---------------- 导航点 ----------------
class WaypointIn(BaseModel):
    name: str | None = None
    x: float
    y: float
    yaw: float = 0.0


class WaypointUpdate(BaseModel):
    name: str | None = None
    x: float | None = None
    y: float | None = None
    yaw: float | None = None


@app.get('/api/waypoints')
def list_waypoints(_user: dict = _viewer):
    return waypoints.list()


@app.post('/api/waypoints')
def add_waypoint(wp: WaypointIn, _user: dict = _admin):
    return waypoints.add(wp.name, wp.x, wp.y, wp.yaw)


def _dump_unset(model):
    """pydantic v1/v2 兼容的 exclude_unset 导出。"""
    if hasattr(model, 'model_dump'):
        return model.model_dump(exclude_unset=True)
    return model.dict(exclude_unset=True)


def _nav_running_warning():
    """导航模式运行中改图：热重载会让 costmap 在行驶中瞬变，附加警告文案。"""
    if pm.status()['mode'] == 'navigation':
        return ' ⚠️ 导航模式正在运行：请确认车辆静止，热重载瞬间 costmap 会整体刷新'
    return ''


@app.put('/api/waypoints/{wp_id}')
def update_waypoint(wp_id: int, wp: WaypointUpdate, _user: dict = _admin):
    w = waypoints.update(wp_id, **_dump_unset(wp))
    if w is None:
        raise HTTPException(404, '导航点不存在')
    return w


@app.delete('/api/waypoints/{wp_id}')
def delete_waypoint(wp_id: int, _user: dict = _admin):
    if not waypoints.delete(wp_id):
        raise HTTPException(404, '导航点不存在')
    return {'ok': True}


# ---------------- rosbridge 鉴权代理 ----------------
# 浏览器无法给 WebSocket 加 Authorization 头，token 走查询参数（与登录同源 HTTPS
# 时不会泄露给第三方；明文 HTTP 下与 REST 头同样可被嗅探，故建议开 TLS）。
ROSBRIDGE_UPSTREAM = os.environ.get('DIUNIU_ROSBRIDGE_UPSTREAM', 'ws://127.0.0.1:9090')


@app.websocket('/ws/rosbridge')
async def rosbridge_proxy(ws: WebSocket):
    """JWT 校验通过后，把客户端帧双向转发到本机 rosbridge。

    任意登录角色（viewer+）都可连——ROS 话题读（地图/TF/状态）是监控刚需；
    写类操作的权限约束在 REST 层与任务调度层，rosbridge 协议本身无角色概念。
    """
    payload = decode_token(ws.query_params.get('token', ''))
    if payload is None or db.get_user_by_id(int(payload['sub'])) is None:
        await ws.close(code=4401)
        return
    await ws.accept()
    try:
        async with websockets.connect(ROSBRIDGE_UPSTREAM, max_size=None) as up:

            async def client_to_up():
                while True:
                    msg = await ws.receive()
                    if msg.get('type') == 'websocket.disconnect':
                        return
                    if msg.get('text') is not None:
                        await up.send(msg['text'])
                    elif msg.get('bytes') is not None:
                        await up.send(msg['bytes'])

            async def up_to_client():
                async for data in up:
                    if isinstance(data, bytes):
                        await ws.send_bytes(data)
                    else:
                        await ws.send_text(data)

            tasks = {asyncio.create_task(client_to_up()),
                     asyncio.create_task(up_to_client())}
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for t in pending:
                t.cancel()
    except Exception:
        pass
    finally:
        try:
            await ws.close()
        except Exception:
            pass


# ---------------- 前端静态页（Vue SPA，history 模式回退 index.html） ----------------
@app.get('/{full_path:path}')
def spa(full_path: str):
    if full_path.startswith('api/'):
        raise HTTPException(404)
    web_abs = os.path.abspath(WEB_DIR)
    fpath = os.path.abspath(os.path.join(web_abs, full_path))
    if full_path and os.path.commonpath([web_abs, fpath]) == web_abs and os.path.isfile(fpath):
        return FileResponse(fpath)
    return FileResponse(os.path.join(web_abs, 'index.html'))



def main():
    # 可选 TLS：设置 DIUNIU_TLS_CERT / DIUNIU_TLS_KEY 即启用 HTTPS/WSS，
    # 否则明文 HTTP（局域网可信网段内可用，跨网段务必开 TLS）
    tls = {}
    cert, key = os.environ.get('DIUNIU_TLS_CERT'), os.environ.get('DIUNIU_TLS_KEY')
    if cert and key:
        tls = {'ssl_certfile': cert, 'ssl_keyfile': key}
    uvicorn.run(app, host='0.0.0.0', port=8000, log_level='info', **tls)


if __name__ == '__main__':
    main()
