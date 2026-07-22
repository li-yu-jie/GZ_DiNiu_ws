/* =============================================================================
 * app.js — 主装配：工具切换 / 模式控制 / 建图保存流水线进度
 * ==========================================================================*/
(function () {
  const $ = id => document.getElementById(id);

  // ---------- 工具切换 ----------
  const toolBtns = document.querySelectorAll('#toolbar .tool[data-tool]');
  App.setTool = function (tool) {
    // 离开编辑器工具而未保存 → 提示放弃
    if (App.editorActive && App.editorActive() && tool !== App.mapview.tool) {
      App.exitEditor();
      App.emit('toast', '已退出编辑（未保存的修改被放弃）');
    }
    App.mapview.tool = tool;
    toolBtns.forEach(b => b.classList.toggle('active', b.dataset.tool === tool));
    const hints = {
      pan: '拖动平移，滚轮/双指缩放',
      goal: '🎯 在地图上按下定点，拖动定朝向，松开发送导航目标',
      initpose: '📍 按下定小车实际位置，拖动定朝向，松开发布初始位姿',
      addwp: '🚩 按下定点，拖动定朝向，松开命名保存导航点',
      editmap: '✏️ 修图模式：直接涂抹地图',
      keepout: '⛔ 禁区模式：涂抹区域禁止通行',
    };
    if (tool === 'editmap' || tool === 'keepout') {
      App.enterEditor(tool);
    } else if (hints[tool]) {
      App.emit('toast', hints[tool]);
    }
  };
  toolBtns.forEach(b => b.addEventListener('click', () => App.setTool(b.dataset.tool)));

  // 缩放按钮
  $('zoom-in').addEventListener('click', () => App.mapview.zoomBy(1.3));
  $('zoom-out').addEventListener('click', () => App.mapview.zoomBy(1 / 1.3));
  $('zoom-fit').addEventListener('click', () => App.mapview.zoomFit());
  $('follow-robot').addEventListener('click', function () {
    App.mapview.follow = !App.mapview.follow;
    this.classList.toggle('active', App.mapview.follow);
  });

  // 面板收起（移动端）
  $('btn-panel-toggle').addEventListener('click', () => {
    if (window.innerWidth <= 768) $('panel').classList.toggle('collapsed');
    else $('panel').classList.toggle('hidden');
  });
  if (window.innerWidth <= 768) $('panel').classList.add('collapsed');

  // ---------- 模式控制 ----------
  async function callMode(mode) {
    try {
      const r = await fetch('/api/mode/' + mode, { method: 'POST' });
      const j = await r.json();
      if (!r.ok) throw new Error(j.detail || r.statusText);
      App.emit('toast', '✅ ' + j.msg);
      App.emit('log', '模式切换: ' + mode);
    } catch (e) {
      App.emit('toast', '❌ ' + e.message);
    }
  }
  $('btn-mode-nav').addEventListener('click', () => {
    if (confirm('启动导航模式？（将加载地图与 Nav2）')) callMode('navigation');
  });
  $('btn-mode-map').addEventListener('click', () => {
    if (confirm('启动建图模式？（导航将停止，请用摇杆遥控建图）')) callMode('mapping');
  });
  $('btn-mode-stop').addEventListener('click', () => {
    if (confirm('停止当前模式所有节点？')) callMode('stop');
  });

  // 模式状态轮询结果 → 顶栏显示
  let prevMode = null;
  App.on('mode-status', st => {
    const label = { stopped: '未启动', mapping: '建图模式运行中', navigation: '导航模式运行中' }[st.mode] || st.mode;
    $('mode-label').textContent = label;
    $('btn-mode-nav').classList.toggle('active', st.mode === 'navigation');
    $('btn-mode-map').classList.toggle('active', st.mode === 'mapping');
    $('btn-save-map').classList.toggle('hidden', st.mode !== 'mapping');
    // 建图模式切换地图渲染为 odom 帧（轨迹+激光）
    App.mapview.setOdomMode(st.mode === 'mapping');
    // 切入导航模式：重订阅 latched /map 拿最新地图 + 刷新禁区叠加
    if (st.mode === 'navigation' && prevMode !== 'navigation') {
      setTimeout(() => {
        App.resubscribe('/map');
        App.mapview.reloadKeepout();
      }, 3000);   // 等 map_server 激活发布
    }
    prevMode = st.mode;
    // 建图模式自动开启摇杆提示
    if (st.mode === 'mapping' && !App.teleopActive() && !App._teleopHinted) {
      App._teleopHinted = true;
      App.emit('toast', '建图模式：点左侧 🕹️ 开启虚拟摇杆遥控');
    }
    if (st.mode !== 'mapping') App._teleopHinted = false;
    // 保存流水线进度
    pollSaveStatus(st.save);
  });

  // ---------- 建图保存 ----------
  let saveWatch = false;
  $('btn-save-map').addEventListener('click', async () => {
    if (!confirm('保存当前建图？\n流程：停止建图 → PCD转栅格图 → 覆盖导航地图 → 自动切回导航模式')) return;
    try {
      const r = await fetch('/api/mapping/save', { method: 'POST' });
      const j = await r.json();
      if (!r.ok) throw new Error(j.detail || r.statusText);
      App.emit('toast', '💾 开始保存地图...');
      saveWatch = true;
    } catch (e) {
      App.emit('toast', '❌ ' + e.message);
    }
  });

  let lastSaveLogLen = 0;
  function pollSaveStatus(save) {
    if (!save) return;
    // 增量打印流水线日志
    if (save.log && save.log.length > lastSaveLogLen) {
      save.log.slice(lastSaveLogLen).forEach(l => App.emit('log', '[保存] ' + l));
      lastSaveLogLen = save.log.length;
    }
    if (save.running) {
      $('btn-save-map').textContent = '💾 ' + (save.step || '保存中...');
      $('btn-save-map').disabled = true;
    } else {
      $('btn-save-map').textContent = '💾 保存地图';
      $('btn-save-map').disabled = false;
      if (saveWatch) {
        saveWatch = false;
        lastSaveLogLen = 0;
        if (save.done) App.emit('toast', '✅ 地图保存完成，已切回导航模式');
        else if (save.error) App.emit('toast', '❌ 保存失败: ' + save.error);
      }
    }
  }

  // ---------- 启动 ROS 话题 ----------
  App.setupRosTopics();
  // 首次收到地图自动适配全图
  let firstMap = true;
  App.on('map-info', () => {
    if (firstMap) { firstMap = false; App.mapview.zoomFit(); }
  });

  App.emit('log', 'Web 控制端已加载');
})();
