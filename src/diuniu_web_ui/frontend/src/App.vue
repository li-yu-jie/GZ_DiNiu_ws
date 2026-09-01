<template>
  <router-view v-if="!auth.isLoggedIn" />
  <el-container v-else class="layout">
    <el-header class="header" height="52px">
      <div class="brand">🚜 地牛叉车 Web 控制端</div>
      <el-menu
        :default-active="$route.path"
        mode="horizontal"
        router
        class="nav-menu"
        :ellipsis="false"
      >
        <el-menu-item index="/">监控</el-menu-item>
        <el-menu-item v-if="auth.isAdmin" index="/mapping">建图</el-menu-item>
        <el-menu-item v-if="auth.isAdmin" index="/mapedit">修图/禁区</el-menu-item>
        <el-menu-item v-if="auth.isAdmin" index="/users">账号</el-menu-item>
      </el-menu>
      <div class="right">
        <el-tag :type="roleTagType" size="small" effect="dark">{{ roleLabel }}</el-tag>
        <el-dropdown @command="onUserCmd">
          <span class="username">
            {{ auth.user?.username }}
            <el-icon><ArrowDown /></el-icon>
          </span>
          <template #dropdown>
            <el-dropdown-menu>
              <el-dropdown-item command="password">修改密码</el-dropdown-item>
              <el-dropdown-item command="logout" divided>退出登录</el-dropdown-item>
            </el-dropdown-menu>
          </template>
        </el-dropdown>
      </div>
    </el-header>
    <el-main class="main">
      <router-view />
    </el-main>

    <el-dialog v-model="pwdVisible" title="修改密码" width="380px">
      <el-form :model="pwdForm" label-width="80px">
        <el-form-item label="原密码">
          <el-input v-model="pwdForm.old_password" type="password" show-password />
        </el-form-item>
        <el-form-item label="新密码">
          <el-input v-model="pwdForm.new_password" type="password" show-password />
        </el-form-item>
      </el-form>
      <template #footer>
        <el-button @click="pwdVisible = false">取消</el-button>
        <el-button type="primary" :loading="pwdLoading" @click="submitPwd">确定</el-button>
      </template>
    </el-dialog>
  </el-container>
</template>

<script setup lang="ts">
import { computed, onMounted, reactive, ref } from 'vue'
import { useRouter } from 'vue-router'
import { ElMessage } from 'element-plus'
import { ArrowDown } from '@element-plus/icons-vue'
import { useAuthStore } from './stores/auth'
import { changePassword } from './api'
import { rosBridge } from './ros/bridge'

const auth = useAuthStore()
const router = useRouter()

const roleLabel = computed(() => ({ admin: '管理员', operator: '操作员', viewer: '观察者' }[auth.role] || auth.role))
const roleTagType = computed(() => ({ admin: 'danger', operator: 'warning', viewer: 'info' }[auth.role] as any || 'info'))

const pwdVisible = ref(false)
const pwdLoading = ref(false)
const pwdForm = reactive({ old_password: '', new_password: '' })

function onUserCmd(cmd: string) {
  if (cmd === 'logout') {
    auth.logout()
    rosBridge.disconnect()
    router.push('/login')
  } else if (cmd === 'password') {
    pwdForm.old_password = ''
    pwdForm.new_password = ''
    pwdVisible.value = true
  }
}

async function submitPwd() {
  pwdLoading.value = true
  try {
    await changePassword(pwdForm.old_password, pwdForm.new_password)
    ElMessage.success('密码已修改')
    pwdVisible.value = false
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '修改失败')
  } finally {
    pwdLoading.value = false
  }
}

// 刷新后恢复会话：校验 token 并建立 rosbridge 连接（第三道防线：登录态才有 WS）
onMounted(async () => {
  if (auth.isLoggedIn) {
    const ok = await auth.fetchMe()
    if (ok) rosBridge.connect()
    else router.push('/login')
  }
})
</script>

<style>
html, body, #app { height: 100%; margin: 0; }
body {
  font-family: 'Helvetica Neue', 'PingFang SC', 'Microsoft YaHei', sans-serif;
  font-size: 14px;
  -webkit-font-smoothing: antialiased;
}
.layout { height: 100%; }
.header {
  display: flex; align-items: center; gap: 28px;
  background: #11161d;
  border-bottom: 1px solid var(--dn-border);
  color: var(--dn-text); padding: 0 20px;
}
.brand {
  font-weight: 600; font-size: 15px; white-space: nowrap; letter-spacing: 0.5px;
  color: #e8eef5; display: flex; align-items: center; gap: 8px;
}
.nav-menu { flex: 1; background: transparent; border-bottom: none; }
.nav-menu .el-menu-item { color: var(--dn-text-dim); border-bottom: 2px solid transparent; transition: color .2s; }
.nav-menu .el-menu-item:hover { color: var(--dn-text); background: transparent; }
.nav-menu .el-menu-item.is-active { color: #fff; border-bottom-color: var(--dn-accent); }
.right { display: flex; align-items: center; gap: 12px; }
.username { color: var(--dn-text); cursor: pointer; display: flex; align-items: center; gap: 4px; }
.username:hover { color: #fff; }
.main { padding: 12px; background: var(--dn-bg); overflow: hidden; }
</style>
