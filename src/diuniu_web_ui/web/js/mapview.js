/* =============================================================================
 * mapview.js — Canvas 地图渲染器
 * 分层绘制：栅格地图 → 禁区叠加 → 全局路径 → 激光 → 导航点 → 机器人 → 手势预览
 * 视图变换：view = {scale(px/m), ox, oy}（屏幕 = 世界×scale + 偏移），y 轴翻转
 * ==========================================================================*/
(function () {
  const canvas = document.getElementById('map-canvas');
  const ctx = canvas.getContext('2d');

  const MV = {
    // 地图数据
    map: null,            // OccupancyGrid msg
    mapImage: null,       // 栅格离屏 canvas（地图原始像素）
    keepoutImg: null,     // 禁区 PNG 图像（原始像素）
    scan: null,           // 最近一次 LaserScan
    plan: null,           // Path
    pose: null,           // {x, y, yaw} map 系
    waypoints: [],
    ghost: null,          // 手势预览 {x, y, yaw}（目标/初始位姿箭头）
    // 视图
    view: { scale: 50, ox: 0, oy: 0 },
    follow: true,
    tool: 'pan',
    dirty: true,
  };
  App.mapview = MV;

  // ---------- 视图变换 ----------
  function worldToScreen(wx, wy) {
    return { x: wx * MV.view.scale + MV.view.ox, y: -wy * MV.view.scale + MV.view.oy };
  }
  function screenToWorld(sx, sy) {
    return { x: (sx - MV.view.ox) / MV.view.scale, y: -(sy - MV.view.oy) / MV.view.scale };
  }
  MV.worldToScreen = worldToScreen;
  MV.screenToWorld = screenToWorld;
  MV.invalidate = () => { MV.dirty = true; };

  function resize() {
    const r = canvas.parentElement.getBoundingClientRect();
    canvas.width = r.width * devicePixelRatio;
    canvas.height = r.height * devicePixelRatio;
    canvas.style.width = r.width + 'px';
    canvas.style.height = r.height + 'px';
    ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
    MV.dirty = true;
  }
  window.addEventListener('resize', resize);

  MV.zoomFit = function () {
    if (!MV.map) return;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    const mw = MV.map.info.width * MV.map.info.resolution;
    const mh = MV.map.info.height * MV.map.info.resolution;
    const s = Math.min(w / mw, h / mh) * 0.95;
    MV.view.scale = s;
    const c = mapCenter();
    MV.view.ox = w / 2 - c.x * s;
    MV.view.oy = h / 2 + c.y * s;
    MV.follow = false;
    document.getElementById('follow-robot').classList.remove('active');
    MV.dirty = true;
  };
  MV.zoomBy = function (f, cx, cy) {
    cx = cx === undefined ? canvas.clientWidth / 2 : cx;
    cy = cy === undefined ? canvas.clientHeight / 2 : cy;
    const s2 = Math.min(500, Math.max(5, MV.view.scale * f));
    MV.view.ox = cx - (cx - MV.view.ox) * (s2 / MV.view.scale);
    MV.view.oy = cy - (cy - MV.view.oy) * (s2 / MV.view.scale);
    MV.view.scale = s2;
    MV.follow = false;
    document.getElementById('follow-robot').classList.remove('active');
    MV.dirty = true;
  };
  function mapCenter() {
    const i = MV.map.info;
    return { x: i.origin.position.x + i.width * i.resolution / 2,
             y: i.origin.position.y + i.height * i.resolution / 2 };
  }

  // ---------- 地图接收与离屏渲染 ----------
  App.on('map', function (msg) {
    MV.lastMapMsg = msg;              // 备份，供退出修图后恢复
    MV.map = msg;
    renderMapImage();
    MV.dirty = true;
    App.emit('map-info', msg.info);
  });
  // 退出修图/禁区编辑后恢复真实栅格（热加载后 map_server 会推新图自动覆盖）
  App.on('editor-exited', function () {
    if (MV.lastMapMsg) {
      MV.map = MV.lastMapMsg;
      renderMapImage();
    }
    MV.dirty = true;
  });
  App.on('scan', m => { MV.scan = m; });
  App.on('plan', m => { MV.plan = m; MV.dirty = true; });
  App.on('pose', p => {
    MV.pose = p;
    if (MV.follow && MV.map) {
      const w = canvas.clientWidth, h = canvas.clientHeight;
      MV.view.ox = w / 2 - p.x * MV.view.scale;
      MV.view.oy = h / 2 + p.y * MV.view.scale;
    }
    MV.dirty = true;
  });

  function renderMapImage() {
    const info = MV.map.info;
    const w = info.width, h = info.height;
    const off = document.createElement('canvas');
    off.width = w; off.height = h;
    const octx = off.getContext('2d');
    const img = octx.createImageData(w, h);
    const d = MV.map.data;
    for (let y = 0; y < h; y++) {
      const row = (h - 1 - y) * w;  // ROS 栅格原点在左下，canvas 在左上
      for (let x = 0; x < w; x++) {
        const v = d[row + x];
        const i = (y * w + x) * 4;
        if (v < 0)      { img.data[i] = 0x3a; img.data[i+1] = 0x3a; img.data[i+2] = 0x3a; img.data[i+3] = 255; }
        else if (v >= 65) { img.data[i] = 0x10; img.data[i+1] = 0x10; img.data[i+2] = 0x18; img.data[i+3] = 255; }
        else            { const g = 235 - v * 2; img.data[i] = g; img.data[i+1] = g; img.data[i+2] = g; img.data[i+3] = 255; }
      }
    }
    octx.putImageData(img, 0, 0);
    MV.mapImage = off;
  }

  // 禁区 PNG 叠加层（编辑器保存后由 editor.js 调用刷新）
  MV.reloadKeepout = function () {
    const im = new Image();
    im.onload = () => { MV.keepoutImg = im; MV.dirty = true; };
    im.onerror = () => { MV.keepoutImg = null; };
    im.src = '/api/keepout/image?t=' + Date.now();
  };

  // ---------- 主绘制 ----------
  function draw() {
    requestAnimationFrame(draw);
    if (!MV.dirty) return;
    MV.dirty = false;

    const W = canvas.clientWidth, H = canvas.clientHeight;
    ctx.fillStyle = '#111';
    ctx.fillRect(0, 0, W, H);
    if (!MV.map || !MV.mapImage) {
      ctx.fillStyle = '#888';
      ctx.font = '16px sans-serif';
      ctx.fillText('等待地图...（请启动导航模式）', 20, 40);
      return;
    }

    const info = MV.map.info;
    const res = info.resolution;
    const ox = info.origin.position.x, oy = info.origin.position.y;

    // 地图位图：世界(m) → 屏幕
    const tl = worldToScreen(ox, oy + info.height * res);
    ctx.imageSmoothingEnabled = MV.view.scale / (1 / res) < 4;
    ctx.drawImage(MV.mapImage, tl.x, tl.y,
                  info.width * res * MV.view.scale, info.height * res * MV.view.scale);

    // 禁区叠加（红色半透明）
    if (MV.keepoutImg) {
      ctx.save();
      ctx.globalAlpha = 0.45;
      ctx.filter = 'sepia(1) saturate(5) hue-rotate(-50deg)';
      ctx.drawImage(MV.keepoutImg, tl.x, tl.y,
                    info.width * res * MV.view.scale, info.height * res * MV.view.scale);
      ctx.restore();
      ctx.filter = 'none';
    }

    // 全局路径（绿）
    if (MV.plan && MV.plan.poses && MV.plan.poses.length > 1) {
      ctx.strokeStyle = '#00e676';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      MV.plan.poses.forEach((p, idx) => {
        const s = worldToScreen(p.pose.position.x, p.pose.position.y);
        idx === 0 ? ctx.moveTo(s.x, s.y) : ctx.lineTo(s.x, s.y);
      });
      ctx.stroke();
    }

    // 激光点（红）
    if (MV.scan && MV.pose) {
      ctx.fillStyle = '#ff5252';
      const a0 = MV.scan.angle_min, da = MV.scan.angle_increment;
      const n = MV.scan.ranges.length;
      const rmin = MV.scan.range_min || 0.15, rmax = MV.scan.range_max || 50;
      const cos = Math.cos(MV.pose.yaw), sin = Math.sin(MV.pose.yaw);
      const step = Math.max(1, Math.floor(n / 720));
      for (let i = 0; i < n; i += step) {
        const r = MV.scan.ranges[i];
        if (!isFinite(r) || r < rmin || r > rmax) continue;
        const a = a0 + i * da;
        const lx = r * Math.cos(a), ly = r * Math.sin(a);
        const wx = MV.pose.x + lx * cos - ly * sin;
        const wy = MV.pose.y + lx * sin + ly * cos;
        const s = worldToScreen(wx, wy);
        ctx.fillRect(s.x - 1.5, s.y - 1.5, 3, 3);
      }
    }

    // 导航点（蓝旗 + 名称）
    MV.waypoints.forEach(wp => {
      const s = worldToScreen(wp.x, wp.y);
      ctx.fillStyle = '#40c4ff';
      ctx.beginPath();
      ctx.arc(s.x, s.y, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#062a3a';
      ctx.font = 'bold 10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(wp.id, s.x, s.y + 3.5);
      ctx.fillStyle = '#b3e5fc';
      ctx.font = '11px sans-serif';
      ctx.fillText(wp.name, s.x, s.y - 11);
      ctx.textAlign = 'left';
    });

    // 手势预览箭头（目标点/初始位姿）
    if (MV.ghost) drawRobotArrow(MV.ghost, '#ffd54f', true);
    // 机器人（青）
    if (MV.pose) drawRobotArrow(MV.pose, '#00e5ff', false);
  }

  function drawRobotArrow(p, color, dashed) {
    const s = worldToScreen(p.x, p.y);
    const L = 16, Wd = 10;
    ctx.save();
    ctx.translate(s.x, s.y);
    ctx.rotate(-p.yaw);
    if (dashed) ctx.setLineDash([4, 3]);
    ctx.strokeStyle = color;
    ctx.fillStyle = color + '66';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(L, 0); ctx.lineTo(-Wd * 0.6, Wd); ctx.lineTo(-Wd * 0.6, -Wd);
    ctx.closePath();
    ctx.fill(); ctx.stroke();
    ctx.restore();
  }

  // ---------- 交互：平移 / 缩放 / 工具回调 ----------
  let pointers = new Map();
  let lastPinch = 0;
  let dragStart = null;

  canvas.addEventListener('pointerdown', e => {
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pointers.size === 1) {
      dragStart = { x: e.clientX, y: e.clientY };
      App.emit('canvas-down', screenToWorld(e.clientX - canvas.getBoundingClientRect().left,
                                            e.clientY - canvas.getBoundingClientRect().top));
    }
  });
  canvas.addEventListener('pointermove', e => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
    if (!pointers.has(e.pointerId)) return;
    const prev = pointers.get(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });

    if (pointers.size === 2) {
      // 双指捏合缩放
      const pts = [...pointers.values()];
      const d = Math.hypot(pts[0].x - pts[1].x, pts[0].y - pts[1].y);
      if (lastPinch > 0) MV.zoomBy(d / lastPinch, sx, sy);
      lastPinch = d;
      return;
    }
    if (pointers.size !== 1) return;

    const w = screenToWorld(sx, sy);
    App.emit('canvas-move', w);

    if (MV.tool === 'pan') {
      MV.view.ox += e.clientX - prev.x;
      MV.view.oy += e.clientY - prev.y;
      MV.follow = false;
      document.getElementById('follow-robot').classList.remove('active');
      MV.dirty = true;
    }
  });
  function pointerUp(e) {
    const rect = canvas.getBoundingClientRect();
    const w = screenToWorld(e.clientX - rect.left, e.clientY - rect.top);
    pointers.delete(e.pointerId);
    lastPinch = 0;
    if (pointers.size === 0 && dragStart) {
      App.emit('canvas-up', w);
      dragStart = null;
    }
  }
  canvas.addEventListener('pointerup', pointerUp);
  canvas.addEventListener('pointercancel', pointerUp);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    MV.zoomBy(e.deltaY < 0 ? 1.15 : 1 / 1.15, e.clientX - rect.left, e.clientY - rect.top);
  }, { passive: false });

  // ---------- 启动 ----------
  App.on('ros-connected', () => MV.reloadKeepout());
  resize();
  requestAnimationFrame(draw);
})();
