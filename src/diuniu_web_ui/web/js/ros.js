/* =============================================================================
 * ros.js — rosbridge 连接管理 + ROS 话题/服务/Action 封装
 * 全局对象 App.ros / App.emit / App.on（极简事件总线）
 * ==========================================================================*/
window.App = window.App || {};

(function () {
  // ---- 极简事件总线 ----
  const handlers = {};
  App.on = function (evt, cb) { (handlers[evt] = handlers[evt] || []).push(cb); };
  App.emit = function (evt, data) {
    (handlers[evt] || []).forEach(cb => { try { cb(data); } catch (e) { console.error(e); } });
  };

  // ---- rosbridge 连接（自动重连） ----
  let ros = null;
  let connected = false;

  function connect() {
    const url = 'ws://' + location.hostname + ':9090';
    ros = new ROSLIB.Ros({ url: url });

    ros.on('connection', () => {
      connected = true;
      App.ros = ros;
      App.emit('ros-connected');
      console.log('[ros] connected');
    });
    ros.on('close', () => {
      connected = false;
      App.emit('ros-disconnected');
      console.log('[ros] closed, retry in 2s');
      setTimeout(connect, 2000);
    });
    ros.on('error', (e) => { console.warn('[ros] error', e); });
  }
  connect();

  App.isConnected = () => connected;

  // ---- 话题订阅管理：断线重连后自动重建 ----
  const subs = [];  // {topic, type, cb, throttle, topicObj}
  App.subscribe = function (topicName, msgType, cb, throttleMs) {
    const entry = { topic: topicName, type: msgType, cb: cb, throttle: throttleMs || 0, topicObj: null };
    subs.push(entry);
    if (connected) bindSub(entry);
  };
  function bindSub(entry) {
    const t = new ROSLIB.Topic({
      ros: ros, name: entry.topic, messageType: entry.type,
      queue_size: 1, throttle_rate: entry.throttle || 0
    });
    t.subscribe(entry.cb);
    entry.topicObj = t;
  }
  App.on('ros-connected', () => { subs.forEach(bindSub); });

  // ---- 常用话题便捷订阅 ----
  App.setupRosTopics = function () {
    // 地图（transient_local，连上就会收到一次）
    App.subscribe('/map', 'nav_msgs/msg/OccupancyGrid', m => App.emit('map', m));
    // 过滤后激光
    App.subscribe('/scan_filtered', 'sensor_msgs/msg/LaserScan', m => App.emit('scan', m), 200);
    // 里程计
    App.subscribe('/odom', 'nav_msgs/msg/Odometry', m => App.emit('odom', m), 200);
    // 全局路径
    App.subscribe('/plan', 'nav_msgs/msg/Path', m => App.emit('plan', m));
    // IMU
    App.subscribe('/imu/data', 'sensor_msgs/msg/Imu', m => App.emit('imu', m), 500);
  };

  // ---- TF：map -> base_link 机器人位姿，10Hz ----
  App.setupTF = function () {
    function makeTF() {
      if (!connected) return;
      const tf = new ROSLIB.TFClient({
        ros: ros, fixedFrame: 'map', angularThres: 0.01, transThres: 0.01, rate: 10
      });
      tf.subscribe('base_link', function (msg) {
        const q = msg.rotation;
        const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        App.emit('pose', { x: msg.translation.x, y: msg.translation.y, yaw: yaw });
      });
      App._tf = tf;
    }
    App.on('ros-connected', makeTF);
    makeTF();
  };

  // ---- 发布器 ----
  App.makePublisher = function (topicName, msgType) {
    let pub = null;
    function ensure() {
      if (connected && (!pub || pub.ros !== ros)) {
        pub = new ROSLIB.Topic({ ros: ros, name: topicName, messageType: msgType, queue_size: 1 });
      }
      return pub;
    }
    return {
      publish: function (msgDict) {
        const p = ensure();
        if (p) p.publish(new ROSLIB.Message(msgDict));
      }
    };
  };

  // ---- Action 客户端 ----
  App.makeActionClient = function (serverName, actionType) {
    let client = null;
    function ensure() {
      if (connected && (!client || client.ros !== ros)) {
        client = new ROSLIB.ActionClient({ ros: ros, serverName: serverName, actionType: actionType });
      }
      return client;
    }
    return {
      sendGoal: function (goalMsg, onFeedback, onResult) {
        const c = ensure();
        if (!c) { App.emit('toast', 'rosbridge 未连接'); return null; }
        const goal = new ROSLIB.Goal({
          actionClient: c,
          goalMessage: goalMsg
        });
        goal.on('feedback', onFeedback);
        goal.on('result', onResult);
        goal.send();
        return goal;
      }
    };
  };
})();
