<template>
  <div class="joy-panel">
    <div class="joy-title">手动遥控</div>
    <div class="joy-ring">
      <div ref="zone" class="joy-zone" />
    </div>
    <div class="joy-info">
      <div class="vel-chip">
        <span class="vel-label">线速度</span>
        <span class="vel-value">{{ vel.linear.toFixed(2) }}<span class="vel-unit">m/s</span></span>
      </div>
      <div class="vel-chip">
        <span class="vel-label">角速度</span>
        <span class="vel-value">{{ vel.angular.toFixed(2) }}<span class="vel-unit">rad/s</span></span>
      </div>
    </div>
    <div class="kbd-hint" :class="{ active: keysActive }">
      <span><kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> / 方向键 行驶</span>
      <span><kbd>Q</kbd><kbd>E</kbd> 升降</span>
      <span><kbd>空格</kbd> 急停</span>
    </div>
    <el-button class="estop" type="danger" size="large" @click="emergencyStop">急 停</el-button>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, reactive, ref } from 'vue'
import nipplejs from 'nipplejs'
import { createPublisher, TOPICS } from '../ros/topics'

const props = withDefaults(defineProps<{
  maxLinear?: number
  maxAngular?: number
}>(), { maxLinear: 0.8, maxAngular: 1.0 })

const zone = ref<HTMLDivElement>()
const vel = reactive({ linear: 0, angular: 0 })

const cmdPub = createPublisher(TOPICS.cmdVelJoy, 'geometry_msgs/Twist')

let manager: any = null
let timer: number | null = null
// 底盘有 0.2s 指令看门狗：必须以 >5Hz 持续发包，否则车会启停抖动。
// 摇杆与键盘各维护一份输入，10Hz 循环发布两者的叠加
const joy = { linear: 0, angular: 0 }
let joyActive = false
const pressed = reactive(new Set<string>())
const keysActive = computed(() => pressed.size > 0)

function clamp(v: number, max: number) {
  return Math.max(-max, Math.min(max, v))
}

// 用 e.code 而非 e.key：与键盘布局无关（WASD 位置固定）
const HANDLED_KEYS = new Set([
  'KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE',
  'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space',
])

function keyVector() {
  const fwd = (pressed.has('KeyW') || pressed.has('ArrowUp') ? 1 : 0)
    - (pressed.has('KeyS') || pressed.has('ArrowDown') ? 1 : 0)
  const turn = (pressed.has('KeyA') || pressed.has('ArrowLeft') ? 1 : 0)
    - (pressed.has('KeyD') || pressed.has('ArrowRight') ? 1 : 0)
  const lift = (pressed.has('KeyQ') ? 1 : 0) - (pressed.has('KeyE') ? 1 : 0)
  return { linear: fwd * props.maxLinear, angular: turn * props.maxAngular, lift }
}

function publishTwist(linear: number, angular: number, lift = 0, angularX = 0) {
  vel.linear = linear
  vel.angular = angular
  try {
    cmdPub.publish({
      linear: { x: linear, y: 0, z: lift },
      angular: { x: angularX, y: 0, z: angular },
    })
  } catch {
    // rosbridge 未连接时静默丢弃
  }
}

function tick() {
  const k = keyVector()
  publishTwist(
    clamp(joy.linear + k.linear, props.maxLinear),
    clamp(joy.angular + k.angular, props.maxAngular),
    k.lift,
  )
}

function startLoop() {
  if (timer !== null) return
  // 10Hz 持续发布当前向量（按住不动时 nipplejs/键盘都不再发事件）
  timer = window.setInterval(tick, 100)
}

function stopLoop() {
  if (timer !== null) {
    clearInterval(timer)
    timer = null
  }
}

// 没有任何输入源时停循环并补一发零速，让看门狗立即安静下来
function settleLoop() {
  if (joyActive || pressed.size > 0) {
    startLoop()
  } else {
    stopLoop()
    publishTwist(0, 0)
  }
}

function emergencyStop() {
  // angular.x > 0.5 是底盘约定的紧急停止通道（STM32 断电）
  publishTwist(0, 0, 0, 1.0)
  joy.linear = 0
  joy.angular = 0
  pressed.clear()
  if (!joyActive) stopLoop()
  manager?.get?.(0)?.restJoystick?.()
}

function isTypingTarget(e: KeyboardEvent) {
  const t = e.target as HTMLElement | null
  return !!t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA'
    || t.tagName === 'SELECT' || t.isContentEditable)
}

function onKeyDown(e: KeyboardEvent) {
  if (isTypingTarget(e) || !HANDLED_KEYS.has(e.code)) return
  e.preventDefault() // 挡住方向键/空格滚动页面
  if (e.code === 'Space') {
    if (!e.repeat) emergencyStop()
    return
  }
  if (!e.repeat) {
    pressed.add(e.code)
    settleLoop()
  }
}

function onKeyUp(e: KeyboardEvent) {
  if (!HANDLED_KEYS.has(e.code)) return
  e.preventDefault()
  if (e.code === 'Space') return
  if (pressed.delete(e.code)) settleLoop()
}

// 窗口失焦时按键事件会丢失，必须清空，否则车会一直跑
function onWindowBlur() {
  if (pressed.size === 0) return
  pressed.clear()
  settleLoop()
}

onMounted(() => {
  manager = nipplejs.create({
    zone: zone.value,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#409eff',
    size: 140,
  })
  manager.on('start', () => { joyActive = true; startLoop() })
  manager.on('move', (_e: any, data: any) => {
    // vector: x 右 / y 上（0~1 归一）；上=前进，左=逆时针
    joy.linear = (data.vector?.y ?? 0) * props.maxLinear
    joy.angular = -(data.vector?.x ?? 0) * props.maxAngular
  })
  manager.on('end', () => {
    joyActive = false
    joy.linear = 0
    joy.angular = 0
    settleLoop()
  })
  window.addEventListener('keydown', onKeyDown)
  window.addEventListener('keyup', onKeyUp)
  window.addEventListener('blur', onWindowBlur)
})

onBeforeUnmount(() => {
  stopLoop()
  publishTwist(0, 0)
  manager?.destroy()
  window.removeEventListener('keydown', onKeyDown)
  window.removeEventListener('keyup', onKeyUp)
  window.removeEventListener('blur', onWindowBlur)
})
</script>

<style scoped>
.joy-panel { display: flex; flex-direction: column; align-items: center; gap: 12px; }
.joy-title {
  color: var(--dn-text); font-size: 13px; font-weight: 600; letter-spacing: 0.5px;
  display: flex; align-items: center; gap: 8px; align-self: flex-start;
}
.joy-title::before {
  content: ''; width: 3px; height: 13px; border-radius: 2px;
  background: var(--dn-accent);
}
.joy-ring {
  width: 178px; height: 178px; border-radius: 50%;
  display: flex; align-items: center; justify-content: center;
  background: var(--dn-surface-2);
  border: 1px solid var(--dn-border-strong);
}
.joy-zone {
  position: relative; /* nipplejs static 模式的定位基准，缺了摇杆会跑出容器 */
  width: 150px; height: 150px; border-radius: 50%;
}
.joy-info { display: flex; gap: 8px; width: 100%; }
.kbd-hint {
  display: flex; flex-direction: column; gap: 4px; width: 100%;
  color: var(--dn-text-faint); font-size: 11px;
}
.kbd-hint.active { color: var(--dn-accent); }
.kbd-hint span { display: flex; align-items: center; gap: 4px; }
.kbd-hint kbd {
  background: var(--dn-surface-2);
  border: 1px solid var(--dn-border-strong);
  border-radius: 3px; padding: 0 4px;
  font-size: 10px; font-family: inherit; color: inherit;
}
.vel-chip {
  flex: 1; display: flex; flex-direction: column; align-items: center; gap: 2px;
  background: var(--dn-surface-2);
  border: 1px solid var(--dn-border);
  border-radius: 6px; padding: 6px 4px;
}
.vel-label { color: var(--dn-text-faint); font-size: 11px; }
.vel-value {
  color: var(--dn-text); font-size: 14px; font-weight: 600;
  font-variant-numeric: tabular-nums;
}
.vel-unit { font-size: 10px; font-weight: 400; color: var(--dn-text-dim); margin-left: 2px; }
.estop {
  width: 100%; font-weight: 700; letter-spacing: 8px; border-radius: 6px;
}
</style>
