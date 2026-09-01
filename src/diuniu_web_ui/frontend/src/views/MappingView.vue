<template>
  <div class="mapping-page">
    <el-card class="card">
      <template #header>
        <div class="card-header">
          <span>建图生命周期管理</span>
          <el-tag :type="mode === 'mapping' ? 'warning' : 'info'">
            {{ mode === 'mapping' ? '建图进行中' : (mode === 'navigation' ? '导航模式' : '已停止') }}
          </el-tag>
        </div>
      </template>

      <el-steps :active="stepActive" align-center finish-status="success" class="steps">
        <el-step title="启动建图" description="启动 FAST-LIO + 雷达驱动" />
        <el-step title="遥控建图" description="摇杆控制叉车走遍区域" />
        <el-step title="结束并保存" description="PCD→栅格图→热加载" />
      </el-steps>

      <div class="actions">
        <el-button
          type="warning" size="large"
          :disabled="mode !== 'stopped' || saving"
          :loading="starting"
          @click="startMapping"
        >启动建图</el-button>
        <el-button
          type="primary" size="large"
          :disabled="mode !== 'mapping' || saving"
          :loading="saving"
          @click="finishAndSave"
        >结束并保存地图</el-button>
        <el-button
          type="info" size="large" plain
          :disabled="mode === 'stopped' || saving"
          @click="stopOnly"
        >仅停止（不保存）</el-button>
      </div>

      <el-alert
        v-if="mode === 'mapping' && !saving"
        type="warning" :closable="false" class="tip"
        title="建图进行中：请回到「监控」页用摇杆低速遥控叉车覆盖整个区域，完成后回来点「结束并保存地图」。"
      />

      <!-- 保存流水线日志 -->
      <div v-if="save.running || save.log.length" class="save-log">
        <div class="log-header">
          <span>保存流水线：{{ save.step || '等待' }}</span>
          <el-tag v-if="save.done" type="success" size="small">完成</el-tag>
          <el-tag v-else-if="save.error" type="danger" size="small">失败</el-tag>
          <el-tag v-else-if="save.running" type="warning" size="small">运行中</el-tag>
        </div>
        <el-scrollbar max-height="240px">
          <pre class="log-text">{{ save.log.join('\n') }}</pre>
        </el-scrollbar>
        <el-alert v-if="save.error" type="error" :title="save.error" :closable="false" />
      </div>
    </el-card>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { getMappingSaveStatus, getStatus, saveMapping, setMode } from '../api'

const mode = ref('stopped')
const starting = ref(false)
const save = ref<any>({ running: false, done: false, error: '', step: '', log: [] })

const saving = computed(() => save.value.running)
const stepActive = computed(() => {
  if (save.value.done) return 3
  if (mode.value === 'mapping') return 2
  return mode.value === 'stopped' ? 0 : 1
})

let timer: number | null = null

async function poll() {
  try {
    const { data } = await getStatus()
    mode.value = data.mode
    save.value = data.save || save.value
  } catch { /* 静默 */ }
}

async function startMapping() {
  starting.value = true
  try {
    await setMode('mapping')
    ElMessage.success('建图模式启动中，请稍候雷达与 SLAM 就绪')
    poll()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '启动失败')
  } finally {
    starting.value = false
  }
}

async function finishAndSave() {
  let name = ''
  try {
    const { value } = await ElMessageBox.prompt(
      '输入新地图名称（留空则覆盖当前地图）', '保存地图',
      { confirmButtonText: '保存', cancelButtonText: '取消', inputPlaceholder: '如：一楼车间' })
    name = (value || '').trim()
  } catch {
    return // 用户取消
  }
  try {
    await saveMapping(name || undefined)
    ElMessage.info('保存流水线已启动，请勿操作车辆')
    poll()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '启动保存失败')
  }
}

async function stopOnly() {
  try {
    await setMode('stop')
    ElMessage.success('已停止')
    poll()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '停止失败')
  }
}

onMounted(() => {
  poll()
  timer = window.setInterval(poll, 1500)
})
onBeforeUnmount(() => { if (timer) clearInterval(timer) })
</script>

<style scoped>
.mapping-page { height: 100%; overflow: auto; padding: 8px; }
.card { max-width: 860px; margin: 0 auto; }
.card-header { display: flex; justify-content: space-between; align-items: center; }
.steps { margin: 16px 0 24px; }
.actions { display: flex; gap: 12px; justify-content: center; margin-bottom: 16px; }
.tip { margin-bottom: 16px; }
.save-log { border-top: 1px solid var(--dn-border); padding-top: 12px; }
.log-header { display: flex; gap: 8px; align-items: center; margin-bottom: 8px; font-weight: 600; }
.log-text {
  background: #0a0e13; color: #8fd6a4; padding: 10px; border-radius: 6px;
  border: 1px solid var(--dn-border);
  font-size: 12px; line-height: 1.7; margin: 0 0 8px; white-space: pre-wrap;
}
</style>
