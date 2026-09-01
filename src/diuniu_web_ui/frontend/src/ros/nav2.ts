// =============================================================================
// nav2.ts — Nav2 Action 客户端与初始位姿发布
//
// 协议说明：
// - roslib 自带的 ActionClient 是 ROS1 actionlib 话题协议，在 ROS 2 下无效；
// - rosbridge 2.x 提供原生 action op（send_action_goal / cancel_action_goal /
//   action_feedback / action_result），但 roslib 的 SocketAdapter 会丢弃这些
//   未知 op，收不到回包；
// - 走 _action/ 底层服务的方案也不可行：rosbridge 的服务类型加载器只认
//   <pkg>/srv/<Name>，无法导入 nav2_msgs/action/*_SendGoal 这类 action 子类型。
// 因此这里为每个导航目标单独开一条原生 WebSocket，直接按 rosbridge action
// 协议收发 JSON（rosbridge 侧已配 send_action_goals_in_new_thread=true，
// 目标处理不会阻塞连接）。
// =============================================================================
import ROSLIB from './roslib-global'
import { rosBridge } from './bridge'
import { createPublisher, TOPICS } from './topics'

export interface NavGoalHandle {
  cancel: () => void
}

function yawToQuaternion(yaw: number) {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
}

function makePose(x: number, y: number, yaw: number) {
  return {
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    pose: {
      position: { x, y, z: 0 },
      orientation: yawToQuaternion(yaw),
    },
  }
}

/** action_msgs/GoalStatus 文案 */
const RESULT_TEXT: Record<number, string> = {
  4: '导航完成',
  5: '导航已取消',
  6: '导航失败（目标不可达）',
}

let goalSeq = 0

/**
 * 经 rosbridge 原生 action 协议下发目标。
 * onResult(ok, msg) 在目标完成/失败/取消/被拒绝时回调（只触发一次）。
 */
function sendActionGoal(
  server: string,
  actionType: string, // 如 'nav2_msgs/action/NavigateToPose'
  goalPayload: any,
  callbacks: {
    onFeedback?: (fb: any) => void
    onResult?: (ok: boolean, msg: string) => void
  } = {},
): NavGoalHandle | null {
  if (!rosBridge.state.connected) return null

  const id = `nav_goal_${Date.now()}_${++goalSeq}`
  let ws: WebSocket
  try {
    ws = new WebSocket(`ws://${location.hostname}:9090`)
  } catch {
    return null
  }

  let finished = false
  let opened = false
  const finish = (ok: boolean, msg: string) => {
    if (finished) return
    finished = true
    callbacks.onResult?.(ok, msg)
    try { ws.close() } catch { /* 忽略 */ }
  }

  ws.onopen = () => {
    opened = true
    ws.send(JSON.stringify({
      op: 'send_action_goal',
      action: server,
      action_type: actionType,
      args: goalPayload,
      id,
      feedback: true,
    }))
  }

  /**
   * 通道断开/出错时尽力而为取消已下发的目标——不取消的话 Nav2 会继续
   * 执行一个前端已经失去跟踪的目标，机器人会"自己跑"。
   */
  const tryCancelOrphanGoal = () => {
    if (!opened || finished) return
    const payload = JSON.stringify({ op: 'cancel_action_goal', action: server, id })
    try {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(payload)
        return
      }
    } catch { /* 尽力而为，忽略 */ }
    // 原通道已死：另开一条短连接发取消（同一 rosbridge，目标 id 全局有效）
    try {
      const cw = new WebSocket(`ws://${location.hostname}:9090`)
      cw.onopen = () => {
        try { cw.send(payload) } catch { /* 尽力而为，忽略 */ }
        try { cw.close() } catch { /* 忽略 */ }
      }
      cw.onerror = () => { /* 尽力而为，忽略 */ }
    } catch { /* 尽力而为，忽略 */ }
  }

  ws.onmessage = (ev) => {
    let msg: any
    try { msg = JSON.parse(ev.data) } catch { return }
    if (msg?.id !== id) return
    if (msg.op === 'action_feedback') {
      callbacks.onFeedback?.(msg.values)
    } else if (msg.op === 'action_result') {
      if (msg.result === false) {
        // rosbridge 侧异常（action server 不存在 / 目标被拒等）
        const detail = typeof msg.values === 'string' ? msg.values : ''
        finish(false, `目标下发失败${detail ? `：${detail}` : ''}（导航模式是否已启动并完成定位？）`)
      } else {
        const status = msg.status
        finish(status === 4, RESULT_TEXT[status as number] || `导航结束 (status=${status})`)
      }
    }
  }

  ws.onerror = () => {
    tryCancelOrphanGoal()
    finish(false, 'action 通道连接失败（rosbridge 异常），导航已中断，已尝试取消目标')
  }
  ws.onclose = () => {
    if (!finished) {
      tryCancelOrphanGoal()
      finish(false, opened
        ? 'action 通道意外断开，导航已中断，已尝试取消目标'
        : '无法连接 rosbridge action 通道')
    }
  }

  return {
    cancel: () => {
      try {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ op: 'cancel_action_goal', action: server, id }))
        }
      } catch { /* 忽略 */ }
    },
  }
}

/**
 * 下发单点导航目标 NavigateToPose。
 * onResult(ok, msg) 在目标完成/失败/被取消时回调。
 */
export function sendNavGoal(
  x: number, y: number, yaw: number,
  callbacks: {
    onFeedback?: (fb: any) => void
    onResult?: (ok: boolean, msg: string) => void
  } = {},
): NavGoalHandle | null {
  return sendActionGoal(
    '/navigate_to_pose',
    'nav2_msgs/action/NavigateToPose',
    { pose: makePose(x, y, yaw) },
    callbacks,
  )
}

/** 沿全部航点依次巡航 NavigateThroughPoses。 */
export function sendPatrolGoal(
  waypoints: { x: number; y: number; yaw: number }[],
  callbacks: { onResult?: (ok: boolean, msg: string) => void } = {},
): NavGoalHandle | null {
  return sendActionGoal(
    '/navigate_through_poses',
    'nav2_msgs/action/NavigateThroughPoses',
    { poses: waypoints.map((w) => makePose(w.x, w.y, w.yaw)) },
    {
      ...callbacks,
      onResult: (ok, msg) =>
        callbacks.onResult?.(ok, ok ? '巡航完成' : msg.replace('导航', '巡航')),
    },
  )
}

const initialPosePub = createPublisher(TOPICS.initialPose, 'geometry_msgs/PoseWithCovarianceStamped')

/** AMCL 重定位：设置初始位姿。 */
export function sendInitialPose(x: number, y: number, yaw: number) {
  initialPosePub.publish({
    header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
    pose: {
      pose: {
        position: { x, y, z: 0 },
        orientation: yawToQuaternion(yaw),
      },
      covariance: new Array(36).fill(0).map((_, i) => (i === 0 || i === 7 || i === 35 ? 0.25 : 0)),
    },
  })
}
