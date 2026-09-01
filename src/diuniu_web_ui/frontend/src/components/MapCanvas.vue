<template>
  <div ref="wrap" class="map-wrap">
    <canvas
      ref="canvas"
      @mousedown="onMouseDown"
      @mousemove="onMouseMove"
      @mouseup="onMouseUp"
      @mouseleave="dragging = false"
      @wheel.prevent="onWheel"
    />
    <div v-if="!hasMap" class="map-hint">
      {{ rosState.connected ? '等待 /map 地图数据…（导航模式启动后自动加载）' : 'rosbridge 未连接' }}
    </div>
    <div v-if="interaction !== 'none'" class="map-tip">
      <template v-if="pendingPoint">
        已选坐标 ({{ pendingPoint.x.toFixed(2) }}, {{ pendingPoint.y.toFixed(2) }}, {{ (pendingPoint.yaw * 180 / Math.PI).toFixed(0) }}°)
        <el-button
          size="small"
          type="primary"
          class="pose-btn"
          @click.stop="confirmPendingDirectly"
        >⚡ 直接下发 (x,y,yaw)</el-button>
      </template>
      <template v-else-if="interaction === 'waypoint-pick'">点击地图选择导航点位置</template>
      <template v-else-if="interaction === 'goal'">点击地图选择目标点</template>
      <template v-else-if="interaction === 'initialpose'">点击地图设置初始位姿</template>
      <el-button
        v-if="robotPose"
        size="small"
        type="success"
        class="pose-btn"
        @click.stop="useRobotPose"
      >🎯 使用当前车位 ({{ robotPose.x.toFixed(2) }}, {{ robotPose.y.toFixed(2) }})</el-button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { rosBridge } from '../ros/bridge'
import { subscribeRobotPose, subscribeTopic, TOPICS } from '../ros/topics'
import type { Waypoint } from '../api'

interface PendingPoint { x: number; y: number; yaw: number }

const props = withDefaults(defineProps<{
  waypoints?: Waypoint[]
  interaction?: 'none' | 'goal' | 'initialpose' | 'waypoint-pick'
}>(), { waypoints: () => [], interaction: 'none' })

const emit = defineEmits<{
  (e: 'pick', pose: { x: number; y: number; yaw: number }): void
}>()

const wrap = ref<HTMLDivElement>()
const canvas = ref<HTMLCanvasElement>()
const rosState = rosBridge.state

// ---------------- 地图状态 ----------------
interface MapInfo { resolution: number; originX: number; originY: number; width: number; height: number }
let mapInfo: MapInfo | null = null
let mapImage: HTMLCanvasElement | null = null // 已按屏幕方向翻转（y 向下）
const hasMap = ref(false) // 响应式标志：驱动提示文字显隐（mapImage 本身非响应式）

const robotPose = ref<{ x: number; y: number; yaw: number } | null>(null)
let scanMsg: any = null
let planMsg: any = null
const pendingPoint = ref<PendingPoint | null>(null)

// ---------------- 视图变换 ----------------
let scale = 1      // 屏幕 px / 地图 px
let offsetX = 0
let offsetY = 0
let dpr = window.devicePixelRatio || 1

function fitView() {
  if (!mapInfo || !wrap.value) return
  const cw = wrap.value.clientWidth
  const ch = wrap.value.clientHeight
  scale = Math.min(cw / mapInfo.width, ch / mapInfo.height) * 0.95
  offsetX = (cw - mapInfo.width * scale) / 2
  offsetY = (ch - mapInfo.height * scale) / 2
  requestDraw()
}

function worldToScreen(x: number, y: number): [number, number] {
  if (!mapInfo) return [0, 0]
  const px = (x - mapInfo.originX) / mapInfo.resolution
  const py = mapInfo.height - (y - mapInfo.originY) / mapInfo.resolution
  return [px * scale + offsetX, py * scale + offsetY]
}

function screenToWorld(sx: number, sy: number): { x: number; y: number } | null {
  if (!mapInfo) return null
  const px = (sx - offsetX) / scale
  const py = (sy - offsetY) / scale
  return {
    x: mapInfo.originX + px * mapInfo.resolution,
    y: mapInfo.originY + (mapInfo.height - py) * mapInfo.resolution,
  }
}

// ---------------- 绘制 ----------------
let drawScheduled = false
function requestDraw() {
  if (drawScheduled) return
  drawScheduled = true
  requestAnimationFrame(() => { drawScheduled = false; draw() })
}

function draw() {
  const cv = canvas.value
  if (!cv || !wrap.value) return
  const cw = wrap.value.clientWidth
  const ch = wrap.value.clientHeight
  if (cv.width !== cw * dpr || cv.height !== ch * dpr) {
    cv.width = cw * dpr
    cv.height = ch * dpr
    cv.style.width = cw + 'px'
    cv.style.height = ch + 'px'
  }
  const ctx = cv.getContext('2d')!
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.fillStyle = '#0f141a'
  ctx.fillRect(0, 0, cw, ch)
  if (!mapImage || !mapInfo) return

  // 栅格地图
  ctx.imageSmoothingEnabled = false
  ctx.drawImage(mapImage, offsetX, offsetY, mapInfo.width * scale, mapInfo.height * scale)

  // 全局路径（绿）
  if (planMsg?.poses?.length) {
    ctx.strokeStyle = '#2ecc71'
    ctx.lineWidth = 2
    ctx.beginPath()
    planMsg.poses.forEach((p: any, i: number) => {
      const [sx, sy] = worldToScreen(p.pose.position.x, p.pose.position.y)
      i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy)
    })
    ctx.stroke()
  }

  // 激光点云（红，随机器人位姿变换到 map）
  if (scanMsg && robotPose.value) {
    const { x: rx, y: ry, yaw } = robotPose.value
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw)
    ctx.fillStyle = 'rgba(231, 76, 60, 0.8)'
    const ranges: number[] = scanMsg.ranges || []
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i]
      if (!isFinite(r) || r < scanMsg.range_min || r > scanMsg.range_max) continue
      const a = scanMsg.angle_min + i * scanMsg.angle_increment
      const bx = r * Math.cos(a), by = r * Math.sin(a)
      const [sx, sy] = worldToScreen(rx + cosY * bx - sinY * by, ry + sinY * bx + cosY * by)
      ctx.fillRect(sx - 1, sy - 1, 2, 2)
    }
  }

  // 通用三角形绘制辅助函数
  function drawTriangle(
    sx: number, sy: number, angle: number, r: number,
    fillStyle: string, strokeStyle: string = '#ffffff', lineWidth: number = 1.5
  ) {
    const tipX = sx + r * Math.cos(angle)
    const tipY = sy + r * Math.sin(angle)
    const leftX = sx + r * 0.75 * Math.cos(angle + (2.3 * Math.PI / 3))
    const leftY = sy + r * 0.75 * Math.sin(angle + (2.3 * Math.PI / 3))
    const rightX = sx + r * 0.75 * Math.cos(angle - (2.3 * Math.PI / 3))
    const rightY = sy + r * 0.75 * Math.sin(angle - (2.3 * Math.PI / 3))

    ctx.fillStyle = fillStyle
    ctx.beginPath()
    ctx.moveTo(tipX, tipY)
    ctx.lineTo(leftX, leftY)
    ctx.lineTo(rightX, rightY)
    ctx.closePath()
    ctx.fill()
    ctx.strokeStyle = strokeStyle
    ctx.lineWidth = lineWidth
    ctx.stroke()
  }

  // 航点（蓝色指向三角形 + 名称）
  for (const wp of props.waypoints) {
    const [sx, sy] = worldToScreen(wp.x, wp.y)
    const [tx, ty] = worldToScreen(wp.x + Math.cos(wp.yaw), wp.y + Math.sin(wp.yaw))
    const angle = Math.atan2(ty - sy, tx - sx)
    drawTriangle(sx, sy, angle, 10, '#3498db', '#ffffff', 1.5)

    ctx.fillStyle = '#ffffff'
    ctx.font = '12px sans-serif'
    ctx.fillText(wp.name, sx + 10, sy - 10)
  }

  // 待定下发目标点（黄色动态指向三角形）
  if (pendingPoint.value) {
    const { x, y, yaw } = pendingPoint.value
    const [sx, sy] = worldToScreen(x, y)
    const [tx, ty] = worldToScreen(x + Math.cos(yaw), y + Math.sin(yaw))
    const angle = Math.atan2(ty - sy, tx - sx)
    drawTriangle(sx, sy, angle, 12, '#f1c40f', '#ffffff', 2)
  }

  // 机器人（橙色箭头，用世界坐标三点变换避免 y 翻转符号问题）
  if (robotPose.value) {
    const { x, y, yaw } = robotPose.value
    const L = 0.45, W = 0.25
    const pt = (dx: number, dy: number): [number, number] =>
      worldToScreen(x + dx * Math.cos(yaw) - dy * Math.sin(yaw),
                    y + dx * Math.sin(yaw) + dy * Math.cos(yaw))
    const tip = pt(L, 0), left = pt(-L * 0.6, W), right = pt(-L * 0.6, -W)
    ctx.fillStyle = '#e67e22'
    ctx.beginPath()
    ctx.moveTo(tip[0], tip[1]); ctx.lineTo(left[0], left[1]); ctx.lineTo(right[0], right[1])
    ctx.closePath(); ctx.fill()
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1; ctx.stroke()
  }
}

// ---------------- 交互 ----------------
let dragging = false
let downX = 0, downY = 0
let moved = 0

function onMouseDown(e: MouseEvent) {
  dragging = true
  moved = 0
  downX = e.offsetX
  downY = e.offsetY
}

function onMouseMove(e: MouseEvent) {
  if (dragging) {
    moved += Math.abs(e.movementX) + Math.abs(e.movementY)
    offsetX += e.movementX
    offsetY += e.movementY
    requestDraw()
  } else if (pendingPoint.value) {
    const w = screenToWorld(e.offsetX, e.offsetY)
    if (w) {
      const dist = Math.hypot(w.x - pendingPoint.value.x, w.y - pendingPoint.value.y)
      if (dist > 0.05) {
        pendingPoint.value.yaw = Math.atan2(w.y - pendingPoint.value.y, w.x - pendingPoint.value.x)
        requestDraw()
      }
    }
  }
}

function onMouseUp(e: MouseEvent) {
  dragging = false
  // 位移小于阈值视为点击，否则视为平移
  if (moved > 6 || props.interaction === 'none') return
  const w = screenToWorld(e.offsetX, e.offsetY)
  if (!w) return
  if (props.interaction === 'waypoint-pick') {
    emit('pick', { x: w.x, y: w.y, yaw: 0 })
    return
  }
  // goal / initialpose：点击第一下生成待定点，旋转鼠标调整角度，第二次点击（或点击直接下发）提交 (x, y, yaw)
  if (!pendingPoint.value) {
    pendingPoint.value = { x: w.x, y: w.y, yaw: 0 }
  } else {
    const p = pendingPoint.value
    let yaw = Math.atan2(w.y - p.y, w.x - p.x)
    if (Math.hypot(w.x - p.x, w.y - p.y) < 0.05) yaw = p.yaw || 0
    emit('pick', { x: p.x, y: p.y, yaw })
    pendingPoint.value = null
  }
  requestDraw()
}

function onWheel(e: WheelEvent) {
  if (!mapInfo) return
  const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15
  const newScale = Math.min(Math.max(scale * factor, 0.05), 60)
  // 以鼠标为中心缩放
  offsetX = e.offsetX - (e.offsetX - offsetX) * (newScale / scale)
  offsetY = e.offsetY - (e.offsetY - offsetY) * (newScale / scale)
  scale = newScale
  requestDraw()
}

// ---------------- ROS 订阅 ----------------
const unsubs: (() => void)[] = []

function renderOccupancyGrid(msg: any) {
  const { width, height } = msg.info
  const data: number[] = msg.data
  const off = document.createElement('canvas')
  off.width = width
  off.height = height
  const ctx = off.getContext('2d')!
  const img = ctx.createImageData(width, height)
  for (let row = 0; row < height; row++) {
    for (let col = 0; col < width; col++) {
      const v = data[row * width + col]
      // ROS 栅格 row 0 在底部 → 翻到屏幕 y 向下
      const di = ((height - 1 - row) * width + col) * 4
      // 深色主题配色：未知=近黑，空闲=深蓝灰，占用=亮白（与页面暗色风格一致）
      let g: number
      if (v < 0) g = 16                    // 未知（接近底色 #10151c）
      else g = Math.min(255, 44 + Math.round(v * 2.1)) // 0=空闲(深蓝灰 #2c…) 100=占用(亮白)
      img.data[di] = g
      img.data[di + 1] = g + (v < 0 ? 5 : 6)   // 略偏蓝
      img.data[di + 2] = g + (v < 0 ? 12 : 14)
      img.data[di + 3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)
  mapInfo = {
    resolution: msg.info.resolution,
    originX: msg.info.origin.position.x,
    originY: msg.info.origin.position.y,
    width,
    height,
  }
  mapImage = off
  hasMap.value = true
  fitView()
}

function startSubscriptions() {
  stopSubscriptions()
  if (!rosBridge.state.connected) return
  unsubs.push(subscribeTopic(TOPICS.map, 'nav_msgs/OccupancyGrid', renderOccupancyGrid))
  unsubs.push(subscribeTopic(TOPICS.scan, 'sensor_msgs/LaserScan', (m) => { scanMsg = m; requestDraw() }, { throttle_rate: 100 }))
  unsubs.push(subscribeTopic(TOPICS.plan, 'nav_msgs/Path', (m) => { planMsg = m; requestDraw() }))
  unsubs.push(subscribeRobotPose((p) => { robotPose.value = p; requestDraw() }))
}

function stopSubscriptions() {
  unsubs.forEach((u) => u())
  unsubs.length = 0
}

watch(() => rosBridge.state.connected, (c) => { if (c) startSubscriptions() })
watch(() => props.waypoints, () => requestDraw(), { deep: true })
watch(() => props.interaction, () => { pendingPoint.value = null; requestDraw() })

let resizeObserver: ResizeObserver | null = null
onMounted(() => {
  dpr = window.devicePixelRatio || 1
  resizeObserver = new ResizeObserver(() => requestDraw())
  if (wrap.value) resizeObserver.observe(wrap.value)
  startSubscriptions()
})

onBeforeUnmount(() => {
  stopSubscriptions()
  resizeObserver?.disconnect()
})

function useRobotPose() {
  if (!robotPose.value) return
  emit('pick', { ...robotPose.value })
  pendingPoint.value = null
  requestDraw()
}

function confirmPendingDirectly() {
  if (!pendingPoint.value) return
  emit('pick', { ...pendingPoint.value })
  pendingPoint.value = null
  requestDraw()
}

function getRobotPose() {
  return robotPose.value
}

defineExpose({ fitView, getRobotPose })
</script>

<style scoped>
.map-wrap { position: relative; width: 100%; height: 100%; overflow: hidden; background: #0a0e13; }
.map-wrap canvas { display: block; cursor: crosshair; }
.map-hint {
  position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
  color: var(--dn-text-faint); font-size: 14px; pointer-events: none;
}
.map-tip {
  position: absolute; top: 8px; left: 50%; transform: translateX(-50%);
  background: rgba(230, 162, 60, 0.95); color: #1a1207; padding: 4px 12px;
  border-radius: 4px; font-size: 13px; font-weight: 500; display: flex; align-items: center; gap: 8px;
  z-index: 10;
}
.pose-btn {
  padding: 2px 8px; height: 22px; font-size: 12px;
}
</style>
