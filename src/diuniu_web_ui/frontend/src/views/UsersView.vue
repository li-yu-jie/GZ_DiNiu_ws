<template>
  <div class="users-page">
    <el-card class="card">
      <template #header>
        <div class="card-header">
          <span>账号管理</span>
          <el-button type="primary" size="small" @click="openCreate">新建账号</el-button>
        </div>
      </template>

      <el-table :data="users" v-loading="loading">
        <el-table-column prop="id" label="ID" width="60" />
        <el-table-column prop="username" label="用户名" />
        <el-table-column label="角色" width="120">
          <template #default="{ row }">
            <el-tag :type="roleTag(row.role)" size="small" effect="dark">{{ roleLabel(row.role) }}</el-tag>
          </template>
        </el-table-column>
        <el-table-column label="创建时间" width="180">
          <template #default="{ row }">{{ formatTime(row.created_at) }}</template>
        </el-table-column>
        <el-table-column label="操作" width="260">
          <template #default="{ row }">
            <el-button size="small" link type="primary" @click="openRole(row)">改角色</el-button>
            <el-button size="small" link type="warning" @click="openReset(row)">重置密码</el-button>
            <el-button
              size="small" link type="danger"
              :disabled="row.id === auth.user?.id"
              @click="onDelete(row)"
            >删除</el-button>
          </template>
        </el-table-column>
      </el-table>
    </el-card>

    <!-- 新建 -->
    <el-dialog v-model="createVisible" title="新建账号" width="380px">
      <el-form label-width="80px">
        <el-form-item label="用户名"><el-input v-model="createForm.username" /></el-form-item>
        <el-form-item label="密码">
          <el-input v-model="createForm.password" type="password" show-password />
        </el-form-item>
        <el-form-item label="角色">
          <el-select v-model="createForm.role">
            <el-option label="管理员" value="admin" />
            <el-option label="操作员" value="operator" />
            <el-option label="观察者" value="viewer" />
          </el-select>
        </el-form-item>
      </el-form>
      <template #footer>
        <el-button @click="createVisible = false">取消</el-button>
        <el-button type="primary" :loading="submitting" @click="submitCreate">创建</el-button>
      </template>
    </el-dialog>

    <!-- 改角色 -->
    <el-dialog v-model="roleVisible" :title="`修改角色：${currentRow?.username}`" width="320px">
      <el-select v-model="roleValue" style="width: 100%">
        <el-option label="管理员" value="admin" />
        <el-option label="操作员" value="operator" />
        <el-option label="观察者" value="viewer" />
      </el-select>
      <template #footer>
        <el-button @click="roleVisible = false">取消</el-button>
        <el-button type="primary" :loading="submitting" @click="submitRole">确定</el-button>
      </template>
    </el-dialog>

    <!-- 重置密码 -->
    <el-dialog v-model="resetVisible" :title="`重置密码：${currentRow?.username}`" width="320px">
      <el-input v-model="resetValue" type="password" show-password placeholder="新密码（至少 6 位）" />
      <template #footer>
        <el-button @click="resetVisible = false">取消</el-button>
        <el-button type="primary" :loading="submitting" @click="submitReset">确定</el-button>
      </template>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
import { onMounted, reactive, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { createUser, deleteUser, listUsers, resetUserPassword, updateUserRole } from '../api'
import { useAuthStore } from '../stores/auth'

const auth = useAuthStore()
const users = ref<any[]>([])
const loading = ref(false)
const submitting = ref(false)

const createVisible = ref(false)
const roleVisible = ref(false)
const resetVisible = ref(false)
const currentRow = ref<any>(null)
const roleValue = ref('viewer')
const resetValue = ref('')
const createForm = reactive({ username: '', password: '', role: 'viewer' })

const roleLabel = (r: string) => ({ admin: '管理员', operator: '操作员', viewer: '观察者' }[r] || r)
const roleTag = (r: string) => ({ admin: 'danger', operator: 'warning', viewer: 'info' }[r] as any || 'info')
const formatTime = (t: number) => t ? new Date(t * 1000).toLocaleString() : '-'

async function refresh() {
  loading.value = true
  try {
    const { data } = await listUsers()
    users.value = data
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '加载失败')
  } finally {
    loading.value = false
  }
}

function openCreate() {
  createForm.username = ''
  createForm.password = ''
  createForm.role = 'viewer'
  createVisible.value = true
}

async function submitCreate() {
  submitting.value = true
  try {
    await createUser(createForm.username, createForm.password, createForm.role)
    ElMessage.success('已创建')
    createVisible.value = false
    refresh()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '创建失败')
  } finally {
    submitting.value = false
  }
}

function openRole(row: any) {
  currentRow.value = row
  roleValue.value = row.role
  roleVisible.value = true
}

async function submitRole() {
  submitting.value = true
  try {
    await updateUserRole(currentRow.value.id, roleValue.value)
    ElMessage.success('角色已更新')
    roleVisible.value = false
    refresh()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '更新失败')
  } finally {
    submitting.value = false
  }
}

function openReset(row: any) {
  currentRow.value = row
  resetValue.value = ''
  resetVisible.value = true
}

async function submitReset() {
  submitting.value = true
  try {
    await resetUserPassword(currentRow.value.id, resetValue.value)
    ElMessage.success('密码已重置')
    resetVisible.value = false
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '重置失败')
  } finally {
    submitting.value = false
  }
}

async function onDelete(row: any) {
  try {
    await ElMessageBox.confirm(`确认删除账号「${row.username}」？`, '删除确认', { type: 'warning' })
  } catch { return }
  try {
    await deleteUser(row.id)
    ElMessage.success('已删除')
    refresh()
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '删除失败')
  }
}

onMounted(refresh)
</script>

<style scoped>
.users-page { height: 100%; overflow: auto; padding: 8px; }
.card { max-width: 900px; margin: 0 auto; }
.card-header { display: flex; justify-content: space-between; align-items: center; }
</style>
