<template>
  <div class="edit-page">
    <div class="toolbar">
      <el-radio-group v-model="tab" size="small" @change="loadAll">
        <el-radio-button value="map">修图（地图灰度）</el-radio-button>
        <el-radio-button value="keepout">禁区编辑</el-radio-button>
      </el-radio-group>

      <el-divider direction="vertical" />
      <template v-if="tab === 'map'">
        <el-button size="small" :type="brush === 0 ? 'primary' : 'default'" @click="brush = 0">黑·占用</el-button>
        <el-button size="small" :type="brush === 254 ? 'primary' : 'default'" @click="brush = 254">白·空闲</el-button>
        <el-button size="small" :type="brush === 205 ? 'primary' : 'default'" @click="brush = 205">灰·未知</el-button>
      </template>
      <template v-else>
        <el-button size="small" :type="brush === 0 ? 'danger' : 'default'" @click="brush = 0">画禁区</el-button>
        <el-button size="small" :type="brush === 254 ? 'primary' : 'default'" @click="brush = 254">擦除</el-button>
      </template>

      <el-divider direction="vertical" />
      <span class="label">笔刷</span>
      <el-slider v-model="brushSize" :min="2" :max="60" style="width: 120px" />
      <el-button size="small" :disabled="!undoStack.length" @click="undo">撤销</el-button>

      <div class="spacer" />
      <el-button size="small" @click="loadAll">重新加载</el-button>
      <el-button size="small" type="primary" :loading="saving" @click="save">保存并热加载</el-button>
      <el-button v-if="auth.isAdmin" size="small" @click="openMapMgr">地图管理</el-button>
    </div>

    <div ref="stageWrap" class="stage">
      <canvas
        ref="stage"
        @mousedown="onDown"
        @mousemove="onMove"
        @mouseup="onUp"
        @mouseleave="onUp"
      />
      <div v-if="loading" class="loading-mask">地图加载中…</div>
    </div>

    <!-- 地图管理（admin） -->
    <el-dialog v-model="mapMgrVisible" title="地图管理" width="560px">
      <el-table :data="mapList" size="small" v-loading="mapMgrLoading">
        <el-table-column prop="name" label="名称" min-width="120" />
        <el-table-column label="创建时间" width="150">
          <template #default="{ row }">{{ row.created_at.replace('T', ' ') }}</template>
        </el-table-column>
        <el-table-column label="状态" width="70">
          <template #default="{ row }">
            <el-tag v-if="row.id === activeMapId" type="success" size="small">当前</el-tag>
          </template>
        </el-table-column>
        <el-table-column label="操作" width="210">
          <template #default="{ row }">
            <el-button
              size="small" type="primary" plain
              :disabled="row.id === activeMapId"
              @click="onActivateMap(row)"
            >设为当前</el-button>
            <el-button size="small" @click="onRenameMap(row)">重命名</el-button>
            <el-button
              size="small" type="danger" plain
              :disabled="row.id === activeMapId"
              @click="onDeleteMap(row)"
            >删除</el-button>
          </template>
        </el-table-column>
      </el-table>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import {
  activateMap, deleteMap, getKeepoutImage, getMapImage, listMaps,
  renameMap, saveKeepoutImage, saveMapImage, type MapInfo,
} from '../api'
import { useAuthStore } from '../stores/auth'

const auth = useAuthStore()

const tab = ref<'map' | 'keepout'>('map')
const brush = ref(0)          // 灰度值：0 黑 / 254 白 / 205 灰
const brushSize = ref(12)
const saving = ref(false)
const loading = ref(false)

const stageWrap = ref<HTMLDivElement>()
const stage = ref<HTMLCanvasElement>()

// editCanvas：正在编辑的灰度图（修图=地图本体；禁区=mask）
let editCanvas: HTMLCanvasElement | null = null
// baseMap：禁区模式下垫底的地图
let baseMap: HTMLCanvasElement | null = null
let viewScale = 1
const undoStack: ImageData[] = []

let painting = false

async function blobToCanvas(blob: Blob): Promise<HTMLCanvasElement> {
  const img = await createImageBitmap(blob)
  const c = document.createElement('canvas')
  c.width = img.width
  c.height = img.height
  c.getContext('2d')!.drawImage(img, 0, 0)
  return c
}

async function loadAll() {
  loading.value = true
  undoStack.length = 0
  try {
    const { blob } = await getMapImage()
    baseMap = await blobToCanvas(blob)
    if (tab.value === 'map') {
      editCanvas = await blobToCanvas(blob)
    } else {
      try {
        const kb = await getKeepoutImage()
        editCanvas = await blobToCanvas(kb)
      } catch {
        // mask 不存在时生成全白
        editCanvas = document.createElement('canvas')
        editCanvas.width = baseMap.width
        editCanvas.height = baseMap.height
        const ctx = editCanvas.getContext('2d')!
        ctx.fillStyle = '#fefefe'
        ctx.fillRect(0, 0, editCanvas.width, editCanvas.height)
      }
    }
    fitStage()
    draw()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '地图加载失败')
  } finally {
    loading.value = false
  }
}

function fitStage() {
  if (!stageWrap.value || !editCanvas) return
  const cw = stageWrap.value.clientWidth
  const ch = stageWrap.value.clientHeight
  viewScale = Math.min(cw / editCanvas.width, ch / editCanvas.height) * 0.98
}

function draw() {
  const cv = stage.value
  if (!cv || !editCanvas) return
  cv.width = editCanvas.width * viewScale
  cv.height = editCanvas.height * viewScale
  const ctx = cv.getContext('2d')!
  ctx.imageSmoothingEnabled = false
  if (tab.value === 'map') {
    ctx.drawImage(editCanvas, 0, 0, cv.width, cv.height)
  } else {
    // 禁区模式：地图垫底 + mask 半透明红叠加
    if (baseMap) ctx.drawImage(baseMap, 0, 0, cv.width, cv.height)
    ctx.save()
    ctx.globalAlpha = 0.55
    // 临时画板：把 mask 黑像素染红
    const tint = document.createElement('canvas')
    tint.width = editCanvas.width
    tint.height = editCanvas.height
    const tctx = tint.getContext('2d')!
    tctx.drawImage(editCanvas, 0, 0)
    tctx.globalCompositeOperation = 'multiply'
    tctx.fillStyle = '#ff0000'
    tctx.fillRect(0, 0, tint.width, tint.height)
    ctx.drawImage(tint, 0, 0, cv.width, cv.height)
    ctx.restore()
  }
}

function paintAt(e: MouseEvent) {
  if (!editCanvas) return
  const x = Math.floor(e.offsetX / viewScale)
  const y = Math.floor(e.offsetY / viewScale)
  const ctx = editCanvas.getContext('2d')!
  ctx.fillStyle = `rgb(${brush.value},${brush.value},${brush.value})`
  ctx.beginPath()
  ctx.arc(x, y, brushSize.value / 2, 0, Math.PI * 2)
  ctx.fill()
  draw()
}

function snapshot() {
  if (!editCanvas) return
  const ctx = editCanvas.getContext('2d')!
  undoStack.push(ctx.getImageData(0, 0, editCanvas.width, editCanvas.height))
  if (undoStack.length > 10) undoStack.shift()
}

function undo() {
  const prev = undoStack.pop()
  if (!prev || !editCanvas) return
  editCanvas.getContext('2d')!.putImageData(prev, 0, 0)
  draw()
}

function onDown(e: MouseEvent) {
  painting = true
  snapshot()
  paintAt(e)
}
function onMove(e: MouseEvent) {
  if (painting) paintAt(e)
}
function onUp() { painting = false }

async function save() {
  if (!editCanvas) return
  saving.value = true
  try {
    const blob: Blob = await new Promise((resolve, reject) =>
      editCanvas!.toBlob((b) => (b ? resolve(b) : reject(new Error('导出 PNG 失败'))), 'image/png'))
    const fn = tab.value === 'map' ? saveMapImage : saveKeepoutImage
    const { data } = await fn(blob)
    ElMessage.success(data.msg || '已保存')
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '保存失败')
  } finally {
    saving.value = false
  }
}

// ---------------- 地图管理（admin） ----------------
const mapMgrVisible = ref(false)
const mapMgrLoading = ref(false)
const mapList = ref<MapInfo[]>([])
const activeMapId = ref('')

async function refreshMaps() {
  mapMgrLoading.value = true
  try {
    const { data } = await listMaps()
    mapList.value = data.maps
    activeMapId.value = data.active
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '地图列表加载失败')
  } finally {
    mapMgrLoading.value = false
  }
}

function openMapMgr() {
  mapMgrVisible.value = true
  refreshMaps()
}

async function onActivateMap(row: MapInfo) {
  try {
    const { data } = await activateMap(row.id)
    ElMessage.success(data.msg || '已切换')
    await refreshMaps()
    loadAll()   // 当前图已变，刷新编辑画布
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '切换失败')
  }
}

async function onRenameMap(row: MapInfo) {
  try {
    const { value } = await ElMessageBox.prompt('新名称', '重命名地图', {
      inputValue: row.name, confirmButtonText: '确定', cancelButtonText: '取消',
    })
    const name = (value || '').trim()
    if (!name || name === row.name) return
    await renameMap(row.id, name)
    ElMessage.success('已重命名')
    refreshMaps()
  } catch (e: any) {
    if (e !== 'cancel') ElMessage.error(e?.response?.data?.detail || '重命名失败')
  }
}

async function onDeleteMap(row: MapInfo) {
  try {
    await ElMessageBox.confirm(`确定删除地图「${row.name}」？文件将一并删除，不可恢复。`, '删除地图', {
      type: 'warning', confirmButtonText: '删除', cancelButtonText: '取消',
    })
  } catch { return }
  try {
    await deleteMap(row.id)
    ElMessage.success('已删除')
    refreshMaps()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '删除失败')
  }
}

let resizeObserver: ResizeObserver | null = null
onMounted(() => {
  loadAll()
  resizeObserver = new ResizeObserver(() => { fitStage(); draw() })
  if (stageWrap.value) resizeObserver.observe(stageWrap.value)
})
onBeforeUnmount(() => resizeObserver?.disconnect())
</script>

<style scoped>
.edit-page { display: flex; flex-direction: column; height: 100%; gap: 10px; }
.toolbar {
  display: flex; align-items: center; gap: 8px; padding: 8px 12px;
  background: var(--dn-surface); border: 1px solid var(--dn-border);
  border-radius: var(--dn-radius); flex-wrap: wrap;
}
.label { color: var(--dn-text-dim); font-size: 12px; }
.spacer { flex: 1; }
.stage {
  flex: 1; min-height: 0; position: relative; overflow: auto;
  background: #0a0e13; border: 1px solid var(--dn-border);
  border-radius: var(--dn-radius);
  display: flex; align-items: flex-start; justify-content: center; padding: 8px;
}
.stage canvas { cursor: crosshair; background: #3a4149; }
.loading-mask {
  position: absolute; inset: 0; display: flex; align-items: center; justify-content: center;
  color: var(--dn-text-dim); background: rgba(10, 14, 19, 0.72);
}
</style>
