/* =============================================================================
 * teleop.js — 虚拟摇杆：发布 geometry_msgs/Twist 到 /cmd_vel_joy
 * 上/下 = 线速度，左/右 = 角速度（底盘双通道隔离，静止时转向只摆轮不走车）
 * 松开立即停止；10Hz 持续发布（底盘看门狗 0.2s 超时自动锁死）
 * ==========================================================================*/
(function () {
  const zone = document.getElementById('joystick-zone');
  const base = document.getElementById('joystick-base');
  const knob = document.getElementById('joystick-knob');

  const MAX_LIN = 0.8;   // m/s，与导航 desired_linear_vel 对齐
  const MAX_ANG = 1.2;   // rad/s
  const RADIUS = 39;     // 摇杆活动半径 (130-52)/2

  const joyPub = App.makePublisher('/cmd_vel_joy', 'geometry_msgs/msg/Twist');

  let active = false;
  let vx = 0, wz = 0;
  let timer = null;

  function setEnabled(on) {
    active = on;
    zone.classList.toggle('hidden', !on);
    document.getElementById('tool-teleop').classList.toggle('active', on);
    if (on) {
      timer = setInterval(() => joyPub.publish(makeTwist(vx, wz)), 100);
      App.emit('toast', '🕹️ 虚拟摇杆已启用（与手柄同通道，勿同时使用）');
    } else {
      clearInterval(timer);
      joyPub.publish(makeTwist(0, 0));
      resetKnob();
      vx = wz = 0;
    }
  }
  App.setTeleop = setEnabled;
  App.teleopActive = () => active;

  function makeTwist(lin, ang) {
    return { linear: { x: lin, y: 0, z: 0 }, angular: { x: 0, y: 0, z: ang } };
  }

  function resetKnob() {
    knob.style.left = '50%';
    knob.style.top = '50%';
  }

  let dragging = false;
  function handleMove(e) {
    if (!dragging) return;
    const rect = base.getBoundingClientRect();
    let dx = e.clientX - (rect.left + rect.width / 2);
    let dy = e.clientY - (rect.top + rect.height / 2);
    const d = Math.hypot(dx, dy);
    if (d > RADIUS) { dx *= RADIUS / d; dy *= RADIUS / d; }
    knob.style.left = 'calc(50% + ' + dx + 'px)';
    knob.style.top = 'calc(50% + ' + dy + 'px)';
    vx = (-dy / RADIUS) * MAX_LIN;   // 上推为正
    wz = (-dx / RADIUS) * MAX_ANG;   // 左推为正（逆时针）
  }

  base.addEventListener('pointerdown', e => {
    dragging = true;
    base.setPointerCapture(e.pointerId);
    handleMove(e);
  });
  base.addEventListener('pointermove', handleMove);
  function release() {
    if (!dragging) return;
    dragging = false;
    vx = wz = 0;
    joyPub.publish(makeTwist(0, 0));
    resetKnob();
  }
  base.addEventListener('pointerup', release);
  base.addEventListener('pointercancel', release);

  document.getElementById('tool-teleop').addEventListener('click', () => setEnabled(!active));
})();
