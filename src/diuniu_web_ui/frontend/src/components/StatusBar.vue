<template>
  <div class="status-bar">
    <div class="top-row">
      <div class="conn" :class="rosState.connected ? 'ok' : 'bad'">
        <span class="dot" />
        rosbridge {{ rosState.connected ? '已连接' : (rosState.connecting ? '连接中…' : '断开') }}
      </div>
      <div class="mode">
        <span class="label">模式</span>
        <el-tag size="small" :type="modeTagType" effect="dark" round>{{ modeLabel }}</el-tag>
      </div>
    </div>
    <div class="tiles">
      <div class="tile">
        <div class="tile-value">{{ sys.cpu_percent?.toFixed(0) ?? '-' }}<span class="unit">%</span></div>
        <div class="tile-label">CPU</div>
      </div>
      <div class="tile">
        <div class="tile-value">{{ sys.mem_percent?.toFixed(0) ?? '-' }}<span class="unit">%</span></div>
        <div class="tile-label">内存</div>
      </div>
      <div class="tile">
        <div class="tile-value">{{ sys.disk_percent?.toFixed(0) ?? '-' }}<span class="unit">%</span></div>
        <div class="tile-label">磁盘</div>
      </div>
      <div class="tile">
        <div class="tile-value">{{ mainTemp !== null ? mainTemp.toFixed(0) : '-' }}<span class="unit">°C</span></div>
        <div class="tile-label">温度</div>
      </div>
      <div class="tile">
        <div class="tile-value">{{ batteryText }}</div>
        <div class="tile-label">电量</div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { getStatus } from '../api'
import { rosBridge } from '../ros/bridge'
import { subscribeTopic, TOPICS } from '../ros/topics'

const rosState = rosBridge.state
const mode = ref('stopped')
const sys = ref<any>({})
const battery = ref<number | null>(null)

const modeLabel = computed(() => ({ stopped: '已停止', mapping: '建图中', navigation: '导航中' }[mode.value] || mode.value))
const modeTagType = computed(() => ({ stopped: 'info', mapping: 'warning', navigation: 'success' }[mode.value] as any || 'info'))
const mainTemp = computed(() => {
  const t = sys.value?.temperatures
  if (!t) return null
  const key = Object.keys(t)[0]
  return key ? t[key] : null
})
const batteryText = computed(() => battery.value === null ? 'N/A' : `${battery.value.toFixed(0)}%`)

let timer: number | null = null
let unsubBattery: (() => void) | null = null

async function poll() {
  try {
    const { data } = await getStatus()
    mode.value = data.mode
    sys.value = data.sysinfo || {}
  } catch {
    // 401 已由拦截器处理；其他错误静默等下一轮
  }
}

watch(() => rosState.connected, (c) => {
  unsubBattery?.()
  unsubBattery = null
  if (c) {
    // 电量话题为可配置占位：无该话题时保持 N/A
    unsubBattery = subscribeTopic(TOPICS.battery, 'sensor_msgs/BatteryState', (m: any) => {
      battery.value = (m.percentage ?? 0) * 100
    }, { throttle_rate: 1000 })
  }
})

onMounted(() => {
  poll()
  timer = window.setInterval(poll, 2000)
})

onBeforeUnmount(() => {
  if (timer) clearInterval(timer)
  unsubBattery?.()
})
</script>

<style scoped>
.status-bar {
  background: var(--dn-surface);
  border: 1px solid var(--dn-border);
  border-radius: var(--dn-radius); padding: 12px 14px;
}
.top-row { display: flex; align-items: center; justify-content: space-between; margin-bottom: 12px; }
.conn { display: flex; align-items: center; gap: 7px; font-size: 12px; color: var(--dn-text-dim); }
.conn.ok { color: var(--dn-success); }
.conn.bad { color: var(--dn-danger); }
.dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; background: currentColor; }
.mode { display: flex; align-items: center; gap: 7px; }
.mode .label { font-size: 12px; color: var(--dn-text-dim); }
.tiles { display: grid; grid-template-columns: repeat(5, 1fr); gap: 8px; }
.tile {
  background: var(--dn-surface-2);
  border: 1px solid var(--dn-border);
  border-radius: 6px; padding: 8px 4px; text-align: center;
}
.tile-value {
  color: var(--dn-text); font-size: 15px; font-weight: 600;
  font-variant-numeric: tabular-nums; line-height: 1.2;
}
.tile-value .unit { font-size: 10px; font-weight: 400; color: var(--dn-text-dim); margin-left: 1px; }
.tile-label { color: var(--dn-text-faint); font-size: 11px; margin-top: 3px; }
</style>
