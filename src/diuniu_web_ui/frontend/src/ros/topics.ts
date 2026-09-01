// =============================================================================
// topics.ts — ROS 话题订阅/发布的统一封装
// =============================================================================
import ROSLIB from './roslib-global'
import { rosBridge } from './bridge'

function requireRos(): any {
  if (!rosBridge.ros || !rosBridge.state.connected) {
    throw new Error('rosbridge 未连接')
  }
  return rosBridge.ros
}

/** 订阅话题，返回取消订阅函数。连接未建立时不抛错，直接返回空操作。 */
export function subscribeTopic(
  name: string,
  messageType: string,
  callback: (msg: any) => void,
  opts: { throttle_rate?: number; queue_length?: number } = {},
): () => void {
  if (!rosBridge.ros) return () => {}
  const topic = new ROSLIB.Topic({
    ros: rosBridge.ros,
    name,
    messageType,
    throttle_rate: opts.throttle_rate ?? 0,
    queue_length: opts.queue_length ?? 1,
  })
  topic.subscribe(callback)
  return () => topic.unsubscribe(callback)
}

/** 创建发布器（懒连接：调用 publish 时才真正要求连接存在）。 */
export function createPublisher(name: string, messageType: string) {
  let topic: any = null
  return {
    publish(msg: any) {
      const ros = requireRos()
      if (!topic) {
        topic = new ROSLIB.Topic({ ros, name, messageType })
      }
      topic.publish(new ROSLIB.Message(msg))
    },
    unadvertise() {
      topic?.unadvertise()
      topic = null
    },
  }
}

// ---------------- 常用话题常量 ----------------
export const TOPICS = {
  map: '/map',
  scan: '/scan_filtered',
  plan: '/plan',
  amclPose: '/amcl_pose',
  cmdVelJoy: '/cmd_vel_joy',
  initialPose: '/initialpose',
  battery: '/battery_state',
  dispatchTask: '/dispatch_task',
} as const

/**
 * 订阅 map→base_link 位姿（两种定位模式都适用），返回取消函数。
 * 不用 ROSLIB.TFClient：该实现依赖 tf2_web_republisher 节点，部署环境没有。
 * 改为直接订阅 /tf 话题，自行缓存并组合 map→odom→base_link 链
 * （建图模式下 laser_mapping 可能直接发 map→base_link，也一并兼容）。
 */
export function subscribeRobotPose(callback: (pose: { x: number; y: number; yaw: number }) => void): () => void {
  if (!rosBridge.ros) return () => {}

  interface TF2 { x: number; y: number; qx: number; qy: number; qz: number; qw: number }
  let mapToOdom: TF2 | null = null
  let odomToBase: TF2 | null = null
  let mapToBase: TF2 | null = null

  const yawOf = (t: TF2) => Math.atan2(2 * (t.qw * t.qz + t.qx * t.qy), 1 - 2 * (t.qy * t.qy + t.qz * t.qz))

  // 平面位姿组合：t = a ∘ b（仅取 yaw 分量，z/roll/pitch 忽略）
  const compose = (a: TF2, b: TF2): TF2 => {
    const yawA = yawOf(a)
    const cos = Math.cos(yawA), sin = Math.sin(yawA)
    const yaw = yawA + yawOf(b)
    return {
      x: a.x + cos * b.x - sin * b.y,
      y: a.y + sin * b.x + cos * b.y,
      qx: 0, qy: 0,
      qz: Math.sin(yaw / 2),
      qw: Math.cos(yaw / 2),
    }
  }

  const emit = () => {
    // 优先用直接的 map→base_link（建图模式），否则组合 map→odom→base_link（AMCL）
    const t = mapToBase ?? (mapToOdom && odomToBase ? compose(mapToOdom, odomToBase) : null)
    if (t) callback({ x: t.x, y: t.y, yaw: yawOf(t) })
  }

  const topic = new ROSLIB.Topic({
    ros: rosBridge.ros,
    name: '/tf',
    messageType: 'tf2_msgs/TFMessage',
    throttle_rate: 50,   // 20Hz 足够 UI 刷新，减轻 rosbridge 压力
    queue_length: 1,
  })
  topic.subscribe((msg: any) => {
    let changed = false
    for (const tr of msg.transforms || []) {
      const parent: string = tr.header?.frame_id || ''
      const child: string = tr.child_frame_id || ''
      const t: TF2 = {
        x: tr.transform.translation.x,
        y: tr.transform.translation.y,
        qx: tr.transform.rotation.x,
        qy: tr.transform.rotation.y,
        qz: tr.transform.rotation.z,
        qw: tr.transform.rotation.w,
      }
      if (parent === 'map' && child === 'odom') { mapToOdom = t; changed = true }
      else if (parent === 'odom' && child === 'base_link') { odomToBase = t; changed = true }
      else if (parent === 'map' && child === 'base_link') { mapToBase = t; changed = true }
    }
    if (changed) emit()
  })
  return () => topic.unsubscribe()
}
