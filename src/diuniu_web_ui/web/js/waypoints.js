/* =============================================================================
 * waypoints.js — 导航点管理：增删改 / 地图标记 / 单点导航
 * ==========================================================================*/
(function () {
  const listEl = document.getElementById('waypoint-list');
  let addingFromMap = null;  // 地图点击待命名的点

  async function refresh() {
    const wps = await fetch('/api/waypoints').then(r => r.json());
    App.mapview.waypoints = wps;
    App.mapview.invalidate();
    renderList(wps);
  }
  App.refreshWaypoints = refresh;

  function renderList(wps) {
    if (!wps.length) {
      listEl.innerHTML = '<em>暂无导航点，点左侧「加点」在地图上添加</em>';
      return;
    }
    listEl.innerHTML = '';
    wps.forEach(wp => {
      const div = document.createElement('div');
      div.className = 'wp-item';
      div.innerHTML =
        '<span class="wp-name" title="点击居中显示">🚩 ' + escapeHtml(wp.name) + '</span>' +
        '<button class="btn" data-act="go">去</button>' +
        '<button class="btn" data-act="rename">改名</button>' +
        '<button class="btn" data-act="here">取当前</button>' +
        '<button class="btn btn-danger" data-act="del">删</button>';
      div.querySelector('.wp-name').addEventListener('click', () => {
        // 地图视图居中到该点
        const canvas = document.getElementById('map-canvas');
        App.mapview.view.ox = canvas.clientWidth / 2 - wp.x * App.mapview.view.scale;
        App.mapview.view.oy = canvas.clientHeight / 2 + wp.y * App.mapview.view.scale;
        App.mapview.follow = false;
        document.getElementById('follow-robot').classList.remove('active');
        App.mapview.invalidate();
      });
      div.querySelector('[data-act=go]').addEventListener('click', () => {
        App.stopPatrol();
        App.sendNavGoal(wp.x, wp.y, wp.yaw);
      });
      div.querySelector('[data-act=rename]').addEventListener('click', async () => {
        const name = prompt('新名称：', wp.name);
        if (name) { await api('PUT', '/api/waypoints/' + wp.id, { name: name }); refresh(); }
      });
      div.querySelector('[data-act=here]').addEventListener('click', async () => {
        const p = App.mapview.pose;
        if (!p) { App.emit('toast', '机器人位姿未知'); return; }
        await api('PUT', '/api/waypoints/' + wp.id, { x: p.x, y: p.y, yaw: p.yaw });
        refresh();
        App.emit('toast', wp.name + ' 已更新为当前位姿');
      });
      div.querySelector('[data-act=del]').addEventListener('click', async () => {
        if (confirm('删除导航点「' + wp.name + '」？')) {
          await api('DELETE', '/api/waypoints/' + wp.id);
          refresh();
        }
      });
      listEl.appendChild(div);
    });
  }

  // ---------- 地图加点手势：按下定点，拖动定朝向，松开命名 ----------
  let gesture = null;
  App.on('canvas-down', w => {
    if (App.mapview.tool === 'addwp') gesture = { x0: w.x, y0: w.y };
  });
  App.on('canvas-move', w => {
    if (!gesture || App.mapview.tool !== 'addwp') return;
    App.mapview.ghost = { x: gesture.x0, y: gesture.y0,
                          yaw: Math.atan2(w.y - gesture.y0, w.x - gesture.x0) };
    App.mapview.invalidate();
  });
  App.on('canvas-up', async w => {
    if (!gesture || App.mapview.tool !== 'addwp') return;
    const x = gesture.x0, y = gesture.y0;
    const yaw = Math.atan2(w.y - gesture.y0, w.x - gesture.x0);
    gesture = null;
    App.mapview.ghost = null;
    App.mapview.invalidate();
    const name = prompt('导航点名称：', '');
    if (name === null) return;
    const r = await api('POST', '/api/waypoints', { name: name || undefined, x: x, y: y, yaw: yaw });
    if (r) {
      App.emit('toast', '🚩 已添加导航点 ' + (r.name || ''));
      refresh();
    }
  });

  async function api(method, url, body) {
    try {
      const r = await fetch(url, {
        method: method,
        headers: body ? { 'Content-Type': 'application/json' } : undefined,
        body: body ? JSON.stringify(body) : undefined
      });
      if (!r.ok) throw new Error((await r.json()).detail || r.statusText);
      return await r.json();
    } catch (e) {
      App.emit('toast', '操作失败: ' + e.message);
      return null;
    }
  }
  App.waypointApi = api;

  function escapeHtml(s) {
    return String(s).replace(/[&<>"']/g, c =>
      ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  }

  App.on('ros-connected', refresh);
  refresh();
})();
