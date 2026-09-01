<template>
  <div class="monitor">
    <div class="map-area">
      <MapCanvas
        ref="mapCanvas"
        :waypoints="waypoints"
        :interaction="interaction"
        @pick="onMapPick"
      />
    </div>

    <div class="side">
      <el-scrollbar class="side-scroll">
        <div class="side-inner">
          <!-- 当前地图：operator+ 可切换 -->
          <div class="block">
            <div class="block-title">当前地图</div>
            <el-select
              v-model="selectedMapId" size="small" style="width: 100%"
              :disabled="!auth.canOperate || switchingMap || !mapList.length"
              :loading="switchingMap"
              @change="onSwitchMap"
            >
              <el-option v-for="m in mapList" :key="m.id" :value="m.id" :label="m.name" />
            </el-select>
          </div>

          <!-- 模式控制：operator+ 可启停导航 -->
          <div class="block" v-if="auth.canOperate">
            <div class="block-title">模式控制</div>
            <div class="btn-row">
              <el-button size="small" type="primary" :disabled="mode === 'navigation'" @click="switchMode('navigation')">
                启动导航
              </el-button>
              <el-button size="small" :disabled="mode === 'stopped'" @click="switchMode('stop')">
                停止
              </el-button>
            </div>
          </div>

          <!-- 自动物流调度 -->
          <div class="block" v-if="auth.canOperate">
            <div class="block-title">自动物流调度</div>
            <div style="margin-bottom: 10px;">
              <el-select v-model="fmsPickupId" placeholder="快速选择预设取货点" size="small" style="width: 100%; margin-bottom: 5px;" @change="onFmsWaypointSelect('pickup')" clearable>
                <el-option v-for="wp in waypoints" :key="wp.id" :label="wp.name" :value="wp.id" />
              </el-select>
              <el-select v-model="fmsDropoffId" placeholder="快速选择预设卸货点" size="small" style="width: 100%;" @change="onFmsWaypointSelect('dropoff')" clearable>
                <el-option v-for="wp in waypoints" :key="wp.id" :label="wp.name" :value="wp.id" />
              </el-select>
            </div>
            <div class="btn-row">
              <el-button
                size="small"
                :type="interaction === 'fms-pickup' ? 'warning' : 'primary'"
                @click="toggleInteraction('fms-pickup')"
              >
                {{ fmsPickup ? '重新选定取货点' : '📍 地图选点 (取)' }}
              </el-button>
              <el-button
                size="small"
                :type="interaction === 'fms-dropoff' ? 'warning' : 'primary'"
                @click="toggleInteraction('fms-dropoff')"
              >
                {{ fmsDropoff ? '重新选定卸货点' : '📦 地图选点 (卸)' }}
              </el-button>
            </div>
            <div style="margin-top: 10px;" v-if="fmsPickup && fmsDropoff">
              <el-button size="small" type="success" style="width: 100%" @click="sendFmsTask">🚀 立即发车</el-button>
            </div>
          </div>

          <!-- 导航操作 -->
          <div class="block" v-if="auth.canOperate">
            <div class="block-title">导航操作</div>
            <div class="btn-row">
              <el-button
                size="small"
                :type="interaction === 'goal' ? 'warning' : 'primary'"
                @click="toggleInteraction('goal')"
              >{{ interaction === 'goal' ? '取消选点' : '设置目标点' }}</el-button>
              <el-button
                size="small"
                :type="interaction === 'initialpose' ? 'warning' : 'default'"
                @click="toggleInteraction('initialpose')"
              >{{ interaction === 'initialpose' ? '取消定位' : '设置初始位姿' }}</el-button>
            </div>
            <el-button
              v-if="navActive" size="small" type="danger" plain class="cancel-btn"
              @click="cancelNav"
            >取消当前导航</el-button>
          </div>

          <!-- 导航点 -->
          <div class="block">
            <WaypointPanel
              :waypoints="waypoints"
              @refresh="loadWaypoints"
              @start-pick="startWaypointPick"
              @pick-current="pickCurrentPose"
              @goto="gotoWaypoint"
              @patrol="startPatrol"
            />
          </div>

          <!-- 摇杆（viewer 不可见） -->
          <div class="block" v-if="auth.canOperate">
            <JoystickPad />
          </div>
        </div>
      </el-scrollbar>
      <StatusBar />
    </div>

    <!-- 航点命名 -->
    <el-dialog v-model="wpDialog" title="保存导航点" width="360px">
      <el-form label-width="70px">
        <el-form-item label="名称">
          <el-input v-model="wpForm.name" placeholder="如：工位A" />
        </el-form-item>
        <el-form-item label="朝向°">
          <el-input-number v-model="wpForm.yawDeg" :min="-180" :max="180" :step="15" />
        </el-form-item>
        <el-form-item label="坐标">
          <div style="display: flex; align-items: center; justify-content: space-between; width: 100%;">
            <span class="coord-text">({{ wpForm.x.toFixed(2) }}, {{ wpForm.y.toFixed(2) }})</span>
            <el-button size="small" type="primary" link @click="fillCurrentPose">填入当前车位</el-button>
          </div>
        </el-form-item>
      </el-form>
      <template #footer>
        <el-button @click="wpDialog = false">取消</el-button>
        <el-button type="primary" @click="saveWaypoint">保存</el-button>
      </template>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
import { onMounted, reactive, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import MapCanvas from '../components/MapCanvas.vue'
import JoystickPad from '../components/JoystickPad.vue'
import WaypointPanel from '../components/WaypointPanel.vue'
import StatusBar from '../components/StatusBar.vue'
import {
  activateMap, addWaypoint, getStatus, listMaps, listWaypoints, setMode,
  type MapInfo, type Waypoint,
} from '../api'
import { sendInitialPose, sendNavGoal, sendPatrolGoal, type NavGoalHandle } from '../ros/nav2'
import { createPublisher, TOPICS } from '../ros/topics'
import { useAuthStore } from '../stores/auth'

const auth = useAuthStore()
const mapCanvas = ref<InstanceType<typeof MapCanvas>>()

const waypoints = ref<Waypoint[]>([])
const mode = ref('stopped')
const interaction = ref<'none' | 'goal' | 'initialpose' | 'waypoint-pick' | 'fms-pickup' | 'fms-dropoff'>('none')
const fmsPickup = ref<{x: number, y: number, yaw: number} | null>(null)
const fmsDropoff = ref<{x: number, y: number, yaw: number} | null>(null)
const fmsPickupId = ref<number | null>(null)
const fmsDropoffId = ref<number | null>(null)
const navActive = ref(false)
let navHandle: NavGoalHandle | null = null

const wpDialog = ref(false)
const wpForm = reactive({ name: '', x: 0, y: 0, yawDeg: 0 })

// 多地图
const mapList = ref<MapInfo[]>([])
const selectedMapId = ref('')   // el-select 的 v-model
let currentMapId = ''           // 服务端确认的激活图（取消/失败时回退用）
const switchingMap = ref(false)

async function loadMaps() {
  try {
    const { data } = await listMaps()
    mapList.value = data.maps
    currentMapId = data.active
    selectedMapId.value = data.active
  } catch { /* 静默 */ }
}

async function onSwitchMap(id: string) {
  if (id === currentMapId) return
  if (mode.value === 'navigation') {
    try {
      await ElMessageBox.confirm(
        '导航正在运行，切换地图将重新加载地图并重置定位，确定切换？',
        '切换地图', { type: 'warning', confirmButtonText: '切换', cancelButtonText: '取消' })
    } catch {
      selectedMapId.value = currentMapId
      return
    }
  }
  switchingMap.value = true
  try {
    const { data } = await activateMap(id)
    currentMapId = id
    ElMessage.success(data.msg || '地图已切换')
    loadWaypoints()              // 航点按图隔离，切图后刷新
    mapCanvas.value?.fitView()   // /map 会由 nav2 重发，MapCanvas 自动重绘
  } catch (e: any) {
    selectedMapId.value = currentMapId
    ElMessage.error(e?.response?.data?.detail || '切换地图失败')
  } finally {
    switchingMap.value = false
  }
}

function toggleInteraction(kind: 'goal' | 'initialpose' | 'fms-pickup' | 'fms-dropoff') {
  interaction.value = interaction.value === kind ? 'none' : kind
}

function onMapPick(pose: { x: number; y: number; yaw: number }) {
  if (interaction.value === 'goal') {
    const h = sendNavGoal(pose.x, pose.y, pose.yaw, {
      onResult: (ok, msg) => {
        navActive.value = false
        navHandle = null
        ok ? ElMessage.success(msg) : ElMessage.warning(msg)
      },
    })
    if (h) {
      navHandle = h
      navActive.value = true
      ElMessage.info(`目标点已下发 (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)})`)
    } else {
      ElMessage.error('rosbridge 未连接，无法下发目标')
    }
  } else if (interaction.value === 'initialpose') {
    sendInitialPose(pose.x, pose.y, pose.yaw)
    ElMessage.success('初始位姿已发布')
  } else if (interaction.value === 'waypoint-pick') {
    wpForm.x = pose.x
    wpForm.y = pose.y
    wpForm.name = ''
    wpForm.yawDeg = 0
    wpDialog.value = true
  } else if (interaction.value === 'fms-pickup') {
    fmsPickup.value = pose
    interaction.value = 'fms-dropoff' // 自动跳到选卸货点
    ElMessage.success('取货点已设定，请在地图上点击卸货点')
    return // 不置空 interaction
  } else if (interaction.value === 'fms-dropoff') {
    fmsDropoff.value = pose
    ElMessage.success('卸货点已设定，可以点击发车了！')
  }
  interaction.value = 'none'
}

const dispatchPub = createPublisher(TOPICS.dispatchTask, 'std_msgs/String')
function sendFmsTask() {
  if (!fmsPickup.value || !fmsDropoff.value) return
  const p = fmsPickup.value
  const d = fmsDropoff.value
  const msg = `PICKUP_COORD: ${p.x.toFixed(3)},${p.y.toFixed(3)},${p.yaw.toFixed(3)}, DROPOFF_COORD: ${d.x.toFixed(3)},${d.y.toFixed(3)},${d.yaw.toFixed(3)}`
  try {
    dispatchPub.publish({ data: msg })
  } catch (e: any) {
    // 发布失败（rosbridge 未连接等）：提示用户并保留已选点位，允许重试
    ElMessage.error(`发车指令下达失败：${e?.message || '未知错误'}，请检查连接后重试`)
    return
  }
  ElMessage.success('🚀 发车指令已下达！')
  fmsPickup.value = null
  fmsDropoff.value = null
  fmsPickupId.value = null
  fmsDropoffId.value = null
}

function onFmsWaypointSelect(type: 'pickup' | 'dropoff') {
  const wpId = type === 'pickup' ? fmsPickupId.value : fmsDropoffId.value
  if (wpId == null) {
    if (type === 'pickup') fmsPickup.value = null
    else fmsDropoff.value = null
    return
  }
  const wp = waypoints.value.find(w => w.id === wpId)
  if (wp) {
    const pose = { x: wp.x, y: wp.y, yaw: wp.yaw }
    if (type === 'pickup') fmsPickup.value = pose
    else fmsDropoff.value = pose
  }
}

function startWaypointPick() {
  interaction.value = 'waypoint-pick'
  ElMessage.info('点击地图选择导航点位置')
}

function fillCurrentPose() {
  const pose = mapCanvas.value?.getRobotPose()
  if (!pose) {
    ElMessage.warning('未能获取车身位姿，请检查 ROS / TF 是否正常')
    return
  }
  wpForm.x = pose.x
  wpForm.y = pose.y
  wpForm.yawDeg = Math.round(pose.yaw * 180 / Math.PI)
  ElMessage.success('已填入当前车身坐标与朝向')
}

function pickCurrentPose() {
  const pose = mapCanvas.value?.getRobotPose()
  if (!pose) {
    ElMessage.warning('未能获取车身位姿，请检查 ROS / TF 是否正常')
    return
  }
  wpForm.x = pose.x
  wpForm.y = pose.y
  wpForm.name = ''
  wpForm.yawDeg = Math.round(pose.yaw * 180 / Math.PI)
  wpDialog.value = true
}

async function saveWaypoint() {
  try {
    await addWaypoint(wpForm.name || '未命名', wpForm.x, wpForm.y, wpForm.yawDeg * Math.PI / 180)
    ElMessage.success('导航点已保存')
    wpDialog.value = false
    loadWaypoints()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '保存失败')
  }
}

function gotoWaypoint(wp: Waypoint) {
  const h = sendNavGoal(wp.x, wp.y, wp.yaw, {
    onResult: (ok, msg) => {
      navActive.value = false
      navHandle = null
      ok ? ElMessage.success(msg) : ElMessage.warning(msg)
    },
  })
  if (h) {
    navHandle = h
    navActive.value = true
    ElMessage.info(`前往「${wp.name}」`)
  } else {
    ElMessage.error('rosbridge 未连接')
  }
}

function startPatrol(wps: Waypoint[]) {
  const h = sendPatrolGoal(wps, {
    onResult: (ok, msg) => {
      navActive.value = false
      navHandle = null
      ok ? ElMessage.success(msg) : ElMessage.warning(msg)
    },
  })
  if (h) {
    navHandle = h
    navActive.value = true
    ElMessage.info('巡航已开始')
  } else {
    ElMessage.error('rosbridge 未连接')
  }
}

function cancelNav() {
  navHandle?.cancel()
  navHandle = null
  navActive.value = false
}

async function switchMode(m: 'navigation' | 'stop') {
  try {
    await setMode(m)
    ElMessage.success(m === 'stop' ? '已停止' : '导航模式启动中')
    refreshMode()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '操作失败')
  }
}

async function loadWaypoints() {
  try {
    const { data } = await listWaypoints()
    waypoints.value = data
  } catch { /* 静默 */ }
}

async function refreshMode() {
  try {
    const { data } = await getStatus()
    mode.value = data.mode
  } catch { /* 静默 */ }
}

onMounted(() => {
  loadMaps()
  loadWaypoints()
  refreshMode()
})
</script>

<style scoped>
.monitor { display: flex; gap: 12px; height: 100%; }
.map-area {
  flex: 1; min-width: 0; border-radius: var(--dn-radius); overflow: hidden;
  border: 1px solid var(--dn-border);
}
.side { width: 316px; display: flex; flex-direction: column; gap: 12px; }
.side-scroll { flex: 1; min-height: 0; }
.side-inner { display: flex; flex-direction: column; gap: 12px; padding-right: 4px; }
.block {
  background: var(--dn-surface);
  border: 1px solid var(--dn-border);
  border-radius: var(--dn-radius); padding: 14px;
}
.block-title {
  color: var(--dn-text); font-size: 13px; font-weight: 600; margin-bottom: 12px;
  display: flex; align-items: center; gap: 8px; letter-spacing: 0.5px;
}
.block-title::before {
  content: ''; width: 3px; height: 13px; border-radius: 2px;
  background: var(--dn-accent);
}
.btn-row { display: flex; gap: 8px; flex-wrap: wrap; }
.btn-row .el-button { flex: 1; margin-left: 0; min-width: 0; }
.cancel-btn { width: 100%; margin-top: 10px; }
.coord-text { color: var(--dn-text-dim); font-size: 13px; }
</style>
