/* =============================================================================
 * panel.js — 右侧状态面板：传感器数据 / 系统状态 / 节点存活 / 消息日志
 * ==========================================================================*/
(function () {
  const $ = id => document.getElementById(id);

  // ---------- 消息日志 ----------
  const logArea = $('log-area');
  App.on('log', msg => {
    const div = document.createElement('div');
    div.textContent = new Date().toLocaleTimeString() + ' ' + msg;
    logArea.prepend(div);
    while (logArea.children.length > 80) logArea.removeChild(logArea.lastChild);
  });

  // ---------- Toast ----------
  let toastTimer = null;
  App.on('toast', msg => {
    const t = $('toast');
    t.textContent = msg;
    t.classList.remove('hidden');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => t.classList.add('hidden'), 3000);
    App.emit('log', msg);
  });

  // ---------- rosbridge 连接状态灯 ----------
  function setDot(ok) {
    const d = $('ros-status');
    d.className = 'dot ' + (ok ? 'dot-green' : 'dot-red');
    d.title = ok ? 'rosbridge 已连接' : 'rosbridge 未连接';
  }
  App.on('ros-connected', () => { setDot(true); App.emit('log', 'rosbridge 已连接'); });
  App.on('ros-disconnected', () => { setDot(false); App.emit('log', 'rosbridge 断开，重连中...'); });

  // ---------- 位姿 / 速度 ----------
  App.on('pose', p => {
    $('sv-pose').textContent =
      'x=' + p.x.toFixed(2) + ' y=' + p.y.toFixed(2) + ' θ=' + (p.yaw * 180 / Math.PI).toFixed(0) + '°';
  });
  App.on('odom', m => {
    $('sv-vel').textContent =
      'v=' + m.twist.twist.linear.x.toFixed(2) + ' m/s, ω=' + m.twist.twist.angular.z.toFixed(2) + ' rad/s';
  });

  // ---------- IMU ----------
  App.on('imu', m => {
    const q = m.orientation;
    const roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / Math.PI;
    const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x)))) * 180 / Math.PI;
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / Math.PI;
    $('sv-imu').textContent = 'R' + roll.toFixed(1) + '° P' + pitch.toFixed(1) + '° Y' + yaw.toFixed(1) + '°';
  });

  // ---------- 激光帧率 ----------
  let scanCount = 0;
  App.on('scan', () => scanCount++);
  setInterval(() => { $('sv-scan').textContent = scanCount + ' Hz'; scanCount = 0; }, 1000);

  // ---------- 系统状态 + 模式轮询 ----------
  async function pollStatus() {
    try {
      const r = await fetch('/api/status');
      const st = await r.json();
      // 系统
      $('sv-cpu').textContent = st.sysinfo.cpu_percent.toFixed(0) + '%  负载 ' + st.sysinfo.load_avg.join('/');
      $('sv-mem').textContent = st.sysinfo.mem_used_gb + '/' + st.sysinfo.mem_total_gb + ' GB (' + st.sysinfo.mem_percent.toFixed(0) + '%)';
      const temps = Object.entries(st.sysinfo.temperatures || {});
      $('sv-temp').textContent = temps.length ? temps.map(([k, v]) => k + ' ' + v + '°C').join(', ') : '--';
      $('sv-disk').textContent = st.sysinfo.disk_percent.toFixed(0) + '%';
      // 模式
      App.emit('mode-status', st);
    } catch (e) { /* 后端未启动时静默 */ }
  }
  setInterval(pollStatus, 2000);
  pollStatus();

  // ---------- 节点存活（rosapi） ----------
  function pollNodes() {
    if (!App.isConnected() || !App.ros) { $('node-list').textContent = 'rosbridge 未连接'; return; }
    App.ros.getNodes(nodes => {
      $('node-list').textContent = nodes.length ? nodes.sort().join('\n') : '--';
      $('node-list').style.whiteSpace = 'pre-line';
    }, () => { $('node-list').textContent = '查询失败'; });
  }
  setInterval(pollNodes, 5000);
})();
