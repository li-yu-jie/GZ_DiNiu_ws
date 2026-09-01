<template>
  <div class="wp-panel">
    <div class="wp-header">
      <span class="wp-title">导航点</span>
      <div class="wp-header-btns">
        <el-button v-if="auth.isAdmin" size="small" type="primary" plain @click="$emit('start-pick')">地图打点</el-button>
        <el-button v-if="auth.isAdmin" size="small" type="success" plain @click="$emit('pick-current')">当前车位</el-button>
        <el-button size="small" plain @click="refresh">刷新</el-button>
      </div>
    </div>
    <el-scrollbar max-height="240px">
      <div v-if="!waypoints.length" class="wp-empty">暂无导航点</div>
      <div v-for="wp in waypoints" :key="wp.id" class="wp-item">
        <div class="wp-pin">
          <svg viewBox="0 0 24 24" width="14" height="14" fill="currentColor">
            <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5A2.5 2.5 0 1 1 12 6.5a2.5 2.5 0 0 1 0 5z"/>
          </svg>
        </div>
        <div class="wp-info">
          <div class="wp-name">{{ wp.name }}</div>
          <div class="wp-coord">({{ wp.x.toFixed(2) }}, {{ wp.y.toFixed(2) }})</div>
        </div>
        <div class="wp-actions">
          <el-button
            v-if="auth.canOperate"
            size="small" type="primary" link
            @click="$emit('goto', wp)"
          >前往</el-button>
          <el-button
            v-if="auth.isAdmin"
            size="small" type="danger" link
            @click="onDelete(wp)"
          >删除</el-button>
        </div>
      </div>
    </el-scrollbar>
    <el-button
      v-if="auth.canOperate && waypoints.length >= 2"
      class="patrol-btn" size="small" type="primary" plain
      @click="$emit('patrol', waypoints)"
    >沿全部航点巡航</el-button>
  </div>
</template>

<script setup lang="ts">
import { ElMessage, ElMessageBox } from 'element-plus'
import { deleteWaypoint, type Waypoint } from '../api'
import { useAuthStore } from '../stores/auth'

const props = defineProps<{ waypoints: Waypoint[] }>()
const emit = defineEmits<{
  (e: 'refresh'): void
  (e: 'start-pick'): void
  (e: 'pick-current'): void
  (e: 'goto', wp: Waypoint): void
  (e: 'patrol', wps: Waypoint[]): void
}>()

const auth = useAuthStore()

function refresh() { emit('refresh') }

async function onDelete(wp: Waypoint) {
  try {
    await ElMessageBox.confirm(`删除导航点「${wp.name}」？`, '确认', { type: 'warning' })
  } catch { return }
  try {
    await deleteWaypoint(wp.id)
    ElMessage.success('已删除')
    emit('refresh')
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '删除失败')
  }
}
</script>

<style scoped>
.wp-panel { display: flex; flex-direction: column; gap: 10px; }
.wp-header { display: flex; justify-content: space-between; align-items: center; }
.wp-title {
  color: var(--dn-text); font-size: 13px; font-weight: 600; letter-spacing: 0.5px;
  display: flex; align-items: center; gap: 8px;
}
.wp-title::before {
  content: ''; width: 3px; height: 13px; border-radius: 2px;
  background: var(--dn-accent);
}
.wp-header-btns { display: flex; gap: 6px; }
.wp-header-btns .el-button { margin-left: 0; }
.wp-empty { color: var(--dn-text-faint); font-size: 12px; text-align: center; padding: 14px 0; }
.wp-item {
  display: flex; align-items: center; gap: 10px; padding: 7px 8px;
  border-radius: 6px; margin-bottom: 4px;
  background: var(--dn-surface-2);
  border: 1px solid transparent;
  transition: background 0.15s, border-color 0.15s;
}
.wp-item:hover {
  background: rgba(64, 158, 255, 0.08);
  border-color: rgba(64, 158, 255, 0.25);
}
.wp-pin {
  width: 26px; height: 26px; border-radius: 50%; flex: none;
  display: flex; align-items: center; justify-content: center;
  color: #79bbff; background: rgba(64, 158, 255, 0.12);
}
.wp-info { flex: 1; min-width: 0; }
.wp-name { color: var(--dn-text); font-size: 13px; font-weight: 600; }
.wp-coord { color: var(--dn-text-dim); font-size: 11px; font-variant-numeric: tabular-nums; }
.wp-actions { display: flex; flex: none; }
.patrol-btn { width: 100%; }
</style>
