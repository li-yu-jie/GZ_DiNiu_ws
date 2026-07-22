/* =============================================================================
 * editor.js — 修图(map.pgm) 与 禁区(keepout mask) 编辑器
 * 直接在地图原始像素空间作画：修图三色(墙=0/空地=254/未知=205)，禁区二值(黑=禁区/白=通行)
 * 保存：画布 → PNG → POST 后端 → 写 pgm + map_server load_map 热加载
 * ==========================================================================*/
(function () {
  const editorBar = document.getElementById('editor-bar');
  const editorTitle = document.getElementById('editor-title');
  const brushUnknownBtn = document.getElementById('brush-unknown');

  // 编辑状态
  let editCanvas = null;    // 原始像素画布 (ImageData 级编辑)
  let editCtx = null;
  let mode = null;          // 'editmap' | 'keepout'
  let brush = 'wall';
  let painting = false;
  let mapInfo = null;       // {width, height, resolution, origin_x, origin_y}

  // 笔刷颜色
  const COLORS = {
    editmap: { wall: 0, free: 254, unknown: 205 },
    keepout: { wall: 0, free: 254, unknown: 254 }   // 禁区模式无“未知”，白=通行
  };

  // ---------- 进入/退出编辑 ----------
  App.enterEditor = async function (which) {
    mode = which;
    App.mapview.follow = false;
    document.getElementById('follow-robot').classList.remove('active');

    const url = which === 'editmap' ? '/api/map/image' : '/api/keepout/image';
    let resp;
    try {
      resp = await fetch(url + '?t=' + Date.now());
      if (!resp.ok) throw new Error((await resp.json()).detail || resp.statusText);
    } catch (e) {
      App.emit('toast', '加载地图失败: ' + e.message);
      App.setTool('pan');
      return;
    }
    mapInfo = {
      width: parseInt(resp.headers.get('X-Map-Width')),
      height: parseInt(resp.headers.get('X-Map-Height')),
      resolution: parseFloat(resp.headers.get('X-Map-Resolution') || App.mapview.map?.info.resolution || 0.05),
      origin_x: parseFloat(resp.headers.get('X-Map-Origin-X') || (App.mapview.map?.info.origin.position.x ?? 0)),
      origin_y: parseFloat(resp.headers.get('X-Map-Origin-Y') || (App.mapview.map?.info.origin.position.y ?? 0)),
    };
    const blob = await resp.blob();
    const bmp = await createImageBitmap(blob);
    editCanvas = document.createElement('canvas');
    editCanvas.width = mapInfo.width;
    editCanvas.height = mapInfo.height;
    editCtx = editCanvas.getContext('2d', { willReadFrequently: true });
    editCtx.drawImage(bmp, 0, 0);

    // 禁区模式：未知灰(205)先归一为白，避免误存
    if (which === 'keepout') normalizeKeepout();

    // 把编辑画布接管到地图渲染层
    App.mapview.mapImage = editCanvas;
    App.mapview.map = {
      info: {
        width: mapInfo.width, height: mapInfo.height, resolution: mapInfo.resolution,
        origin: { position: { x: mapInfo.origin_x, y: mapInfo.origin_y } }
      },
      data: []
    };
    App.mapview.keepoutImg = null;
    App.mapview.zoomFit();
    App.mapview.invalidate();

    editorTitle.textContent = which === 'editmap' ? '✏️ 修图（黑=墙 白=空地 灰=未知）'
                                                  : '⛔ 禁区（黑=禁行 白=通行）';
    brushUnknownBtn.classList.toggle('hidden', which === 'keepout');
    setBrush(which === 'keepout' ? 'wall' : 'wall');
    editorBar.classList.remove('hidden');
    App.emit('log', '进入' + (which === 'editmap' ? '修图' : '禁区') + '编辑');
  };

  function normalizeKeepout() {
    const img = editCtx.getImageData(0, 0, editCanvas.width, editCanvas.height);
    const d = img.data;
    for (let i = 0; i < d.length; i += 4) {
      const v = d[i];
      const out = v < 128 ? 0 : 254;   // 深色→禁区(黑)，其余→通行(白)
      d[i] = d[i + 1] = d[i + 2] = out;
      d[i + 3] = 255;
    }
    editCtx.putImageData(img, 0, 0);
  }

  function exitEditor() {
    mode = null;
    editCanvas = null;
    editCtx = null;
    painting = false;
    editorBar.classList.add('hidden');
    App.mapview.reloadKeepout();
    // 触发重新订阅地图：从 rosbridge 下一次 /map 推送恢复真实栅格
    App.emit('editor-exited');
  }
  App.exitEditor = exitEditor;
  App.editorActive = () => mode !== null;

  // ---------- 笔刷 UI ----------
  function setBrush(b) {
    brush = b;
    document.querySelectorAll('.editor-brush').forEach(el =>
      el.classList.toggle('active', el.dataset.brush === b));
  }
  document.querySelectorAll('.editor-brush').forEach(el =>
    el.addEventListener('click', () => setBrush(el.dataset.brush)));

  // ---------- 作画（屏幕坐标 → 地图原始像素坐标） ----------
  function screenToMapPx(w) {
    const px = Math.floor((w.x - mapInfo.origin_x) / mapInfo.resolution);
    const py = Math.floor(mapInfo.height - (w.y - mapInfo.origin_y) / mapInfo.resolution);
    return { x: px, y: py };
  }

  function paintAt(w) {
    if (!editCtx) return;
    const p = screenToMapPx(w);
    const size = parseInt(document.getElementById('brush-size').value);
    const v = COLORS[mode][brush];
    editCtx.fillStyle = 'rgb(' + v + ',' + v + ',' + v + ')';
    editCtx.beginPath();
    editCtx.arc(p.x, p.y, size, 0, Math.PI * 2);
    editCtx.fill();
    App.mapview.invalidate();
  }

  App.on('canvas-down', w => { if (mode) { painting = true; paintAt(w); } });
  App.on('canvas-move', w => { if (mode && painting) paintAt(w); });
  App.on('canvas-up', () => { painting = false; });

  // ---------- 保存 / 放弃 ----------
  document.getElementById('editor-save').addEventListener('click', async () => {
    if (!editCanvas) return;
    const url = mode === 'editmap' ? '/api/map/save' : '/api/keepout/save';
    App.emit('toast', '保存中并热加载地图...');
    const blob = await new Promise(res => editCanvas.toBlob(res, 'image/png'));
    try {
      const r = await fetch(url, { method: 'POST', body: blob });
      const j = await r.json();
      if (!r.ok) throw new Error(j.detail || r.statusText);
      App.emit('toast', '✅ ' + (j.msg || '保存成功'));
      App.emit('log', (mode === 'editmap' ? '修图' : '禁区') + '保存成功: ' + (j.msg || ''));
    } catch (e) {
      App.emit('toast', '❌ 保存失败: ' + e.message);
    }
    const which = mode;
    exitEditor();
    App.setTool('pan');
  });

  document.getElementById('editor-cancel').addEventListener('click', () => {
    exitEditor();
    App.setTool('pan');
    App.emit('toast', '已放弃修改');
  });
})();
