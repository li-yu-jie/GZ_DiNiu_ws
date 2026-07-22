# =============================================================================
# main.py — FastAPI 后端入口
#
# 职责：托管 web/ 静态前端 + 提供 REST API（模式管理 / 建图保存 / 修图 /
#       禁区 / 导航点 / 系统状态）。ROS 话题与 Action 由前端经 rosbridge 直连。
#
# 启动：ros2 run diuniu_web_ui web_server  （或 launch/web_ui.launch.py 一并起 rosbridge）
# 访问：http://<工控机IP>:8000
# =============================================================================
import os

import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .map_store import MapStore, WaypointStore
from .process_manager import ProcessManager
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
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')


WEB_DIR = _find_web_dir()

app = FastAPI(title='地牛叉车 Web 控制端')
pm = ProcessManager()
maps = MapStore()
waypoints = WaypointStore()


# ---------------- 模式管理 ----------------
@app.post('/api/mode/{mode}')
def set_mode(mode: str):
    if mode == 'stop':
        ok, msg = pm.stop_mode()
    else:
        ok, msg = pm.start_mode(mode)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


@app.get('/api/status')
def status():
    st = pm.status()
    st['sysinfo'] = get_sysinfo()
    st['save'] = {k: v for k, v in pm.save_status.items()}
    return st


# ---------------- 建图保存 ----------------
@app.post('/api/mapping/save')
def mapping_save():
    ok, msg = pm.start_mapping_save()
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


@app.get('/api/mapping/save/status')
def mapping_save_status():
    return pm.save_status


# ---------------- 地图修图 ----------------
@app.get('/api/map/image')
def get_map_image():
    try:
        png, size, y = maps.get_map_image()
    except FileNotFoundError:
        raise HTTPException(404, '地图文件不存在')
    return Response(content=png, media_type='image/png',
                    headers={'X-Map-Width': str(size[0]), 'X-Map-Height': str(size[1]),
                             'X-Map-Resolution': str(y['resolution']),
                             'X-Map-Origin-X': str(y['origin'][0]),
                             'X-Map-Origin-Y': str(y['origin'][1])})


@app.post('/api/map/save')
async def save_map_image(request: Request):
    body = await request.body()
    ok, msg = maps.save_map_image(body)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


# ---------------- 禁区 mask ----------------
@app.get('/api/keepout/image')
def get_keepout_image():
    png, size = maps.get_keepout_image()
    return Response(content=png, media_type='image/png',
                    headers={'X-Map-Width': str(size[0]), 'X-Map-Height': str(size[1])})


@app.post('/api/keepout/save')
async def save_keepout_image(request: Request):
    body = await request.body()
    ok, msg = maps.save_keepout_image(body)
    if not ok:
        raise HTTPException(400, msg)
    return {'ok': True, 'msg': msg}


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
def list_waypoints():
    return waypoints.list()


@app.post('/api/waypoints')
def add_waypoint(wp: WaypointIn):
    return waypoints.add(wp.name, wp.x, wp.y, wp.yaw)


@app.put('/api/waypoints/{wp_id}')
def update_waypoint(wp_id: int, wp: WaypointUpdate):
    w = waypoints.update(wp_id, **wp.dict(exclude_unset=True))
    if w is None:
        raise HTTPException(404, '导航点不存在')
    return w


@app.delete('/api/waypoints/{wp_id}')
def delete_waypoint(wp_id: int):
    if not waypoints.delete(wp_id):
        raise HTTPException(404, '导航点不存在')
    return {'ok': True}


# ---------------- 前端静态页 ----------------
@app.get('/')
def index():
    return FileResponse(os.path.join(WEB_DIR, 'index.html'))


app.mount('/', StaticFiles(directory=WEB_DIR, html=True), name='static')


def main():
    uvicorn.run(app, host='0.0.0.0', port=8000, log_level='info')


if __name__ == '__main__':
    main()
