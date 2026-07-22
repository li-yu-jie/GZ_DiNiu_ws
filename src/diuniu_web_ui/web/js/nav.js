/* =============================================================================
 * nav.js — 导航目标设置 / 初始位姿(重定位) / 取消 / 巡逻
 * 手势：在地图上按下确定位置，拖动决定朝向，松开发送
 * ==========================================================================*/
(function () {
  const MV = () => App.mapview;

  const navClient = App.makeActionClient('/navigate_to_pose', 'nav2_msgs/action/NavigateToPose');
  const patrolClient = App.makeActionClient('/navigate_through_poses', 'nav2_msgs/action/NavigateThroughPoses');
  const initPosePub = App.makePublisher('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped');

  let currentGoal = null;
  let gesture = null;   // {x0, y0} 按下点（世界坐标）
  let patrolActive = false;
  let patrolLoop = false;

  // ---------- 工具手势 ----------
  App.on('canvas-down', w => {
    const tool = MV().tool;
    if (tool === 'goal' || tool === 'initpose') gesture = { x0: w.x, y0: w.y };
  });
  App.on('canvas-move', w => {
    if (!gesture) return;
    MV().ghost = { x: gesture.x0, y: gesture.y0, yaw: Math.atan2(w.y - gesture.y0, w.x - gesture.x0) };
    MV().invalidate();
  });
  App.on('canvas-up', w => {
    if (!gesture) return;
    const tool = MV().tool;
    const yaw = Math.atan2(w.y - gesture.y0, w.x - gesture.x0);
    const gx = gesture.x0, gy = gesture.y0;
    gesture = null;
    MV().ghost = null;
    MV().invalidate();
    if (tool === 'goal') sendGoal(gx, gy, yaw);
    else if (tool === 'initpose') sendInitPose(gx, gy, yaw);
  });

  function poseStamped(x, y, yaw) {
    return {
      header: { frame_id: 'map' },
      pose: {
        position: { x: x, y: y, z: 0.0 },
        orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
      }
    };
  }
  App.poseStamped = poseStamped;

  // ---------- 单点导航 ----------
  function sendGoal(x, y, yaw) {
    cancelGoal(true);
    setNavState('🚀 导航中 → (' + x.toFixed(2) + ', ' + y.toFixed(2) + ')');
    currentGoal = navClient.sendGoal(
      { pose: poseStamped(x, y, yaw) },
      fb => {
        if (fb && fb.distance_remaining !== undefined)
          document.getElementById('nav-feedback').textContent =
            '剩余距离: ' + fb.distance_remaining.toFixed(2) + ' m';
      },
      res => {
        currentGoal = null;
        document.getElementById('btn-cancel-goal').classList.add('hidden');
        document.getElementById('nav-feedback').textContent = '剩余距离: -- m';
        if (patrolActive) { App.emit('patrol-leg-done'); return; }
        setNavState('✅ 到达目标' , true);
      });
    if (currentGoal) document.getElementById('btn-cancel-goal').classList.remove('hidden');
    else setNavState('❌ 目标发送失败', true);
  }
  App.sendNavGoal = sendGoal;

  function cancelGoal(silent) {
    if (currentGoal) { currentGoal.cancel(); currentGoal = null; }
    if (!silent) setNavState('⚠️ 已取消导航', true);
    document.getElementById('btn-cancel-goal').classList.add('hidden');
  }
  App.cancelNavGoal = cancelGoal;

  document.getElementById('btn-cancel-goal').addEventListener('click', () => {
    stopPatrol();
    cancelGoal();
  });

  function setNavState(txt, autoReset) {
    document.getElementById('nav-state').textContent = txt;
    if (autoReset) setTimeout(() => {
      if (!currentGoal && !patrolActive) document.getElementById('nav-state').textContent = '空闲';
    }, 4000);
  }
  App.setNavState = setNavState;

  // ---------- 初始位姿（重定位，替代 relocalize.sh 的 RViz 操作） ----------
  function sendInitPose(x, y, yaw) {
    const msg = poseStamped(x, y, yaw);
    msg.header.stamp = { sec: 0, nanosec: 0 };
    msg.pose.covariance = [
      0.25, 0, 0, 0, 0, 0,
      0, 0.25, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0.0685];
    initPosePub.publish(msg);
    App.emit('toast', '📍 初始位姿已发布: (' + x.toFixed(2) + ', ' + y.toFixed(2) + ')');
    App.emit('log', '发布初始位姿 (' + x.toFixed(2) + ', ' + y.toFixed(2) + ', ' +
             (yaw * 180 / Math.PI).toFixed(0) + '°)');
  }

  // ---------- 多点巡逻 ----------
  async function startPatrol() {
    const wps = await fetch('/api/waypoints').then(r => r.json());
    if (!wps.length) { App.emit('toast', '没有导航点，请先添加'); return; }
    patrolActive = true;
    patrolLoop = document.getElementById('patrol-loop').checked;
    document.getElementById('btn-patrol').classList.add('hidden');
    document.getElementById('btn-stop-patrol').classList.remove('hidden');
    sendPatrolGoal(wps);
  }

  function sendPatrolGoal(wps) {
    setNavState('🔁 巡逻中（' + wps.length + ' 个点）');
    currentGoal = patrolClient.sendGoal(
      { poses: wps.map(wp => poseStamped(wp.x, wp.y, wp.yaw)) },
      fb => {
        if (fb && fb.number_of_poses_remaining !== undefined)
          document.getElementById('nav-feedback').textContent =
            '剩余导航点: ' + fb.number_of_poses_remaining;
      },
      res => {
        currentGoal = null;
        if (!patrolActive) return;
        if (patrolLoop) { sendPatrolGoal(wps); return; }
        stopPatrol();
        setNavState('✅ 巡逻完成', true);
      });
    if (!currentGoal) { stopPatrol(); setNavState('❌ 巡逻启动失败', true); }
  }

  function stopPatrol() {
    patrolActive = false;
    document.getElementById('btn-patrol').classList.remove('hidden');
    document.getElementById('btn-stop-patrol').classList.add('hidden');
  }
  App.stopPatrol = stopPatrol;

  document.getElementById('btn-patrol').addEventListener('click', startPatrol);
  document.getElementById('btn-stop-patrol').addEventListener('click', () => {
    stopPatrol(); cancelGoal();
  });
  App.on('patrol-leg-done', () => { /* NavigateThroughPoses 整体一个 goal，无需逐段处理 */ });
})();
