<template>
  <div class="login-page">
    <div class="login-card">
      <div class="login-logo">🚜</div>
      <div class="login-title">地牛叉车 Web 控制端</div>
      <div class="login-sub">DiNiu Forklift Control</div>
      <el-form :model="form" @keyup.enter="submit">
        <el-form-item>
          <el-input v-model="form.username" placeholder="用户名" :prefix-icon="User" size="large" />
        </el-form-item>
        <el-form-item>
          <el-input v-model="form.password" type="password" placeholder="密码" show-password :prefix-icon="Lock" size="large" />
        </el-form-item>
        <el-button class="login-btn" type="primary" size="large" :loading="loading" @click="submit">
          登 录
        </el-button>
      </el-form>
    </div>
  </div>
</template>

<script setup lang="ts">
import { reactive, ref } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { ElMessage } from 'element-plus'
import { Lock, User } from '@element-plus/icons-vue'
import { login } from '../api'
import { useAuthStore } from '../stores/auth'
import { rosBridge } from '../ros/bridge'

const auth = useAuthStore()
const router = useRouter()
const route = useRoute()

const form = reactive({ username: '', password: '' })
const loading = ref(false)

async function submit() {
  if (!form.username || !form.password) {
    ElMessage.warning('请输入用户名和密码')
    return
  }
  loading.value = true
  try {
    const { data } = await login(form.username, form.password)
    auth.setSession(data.token, data.user)
    // 第三道防线：登录成功后前端才初始化 rosbridge WebSocket
    rosBridge.connect()
    router.push((route.query.redirect as string) || '/')
  } catch (e: any) {
    ElMessage.error(e?.response?.data?.detail || '登录失败')
  } finally {
    loading.value = false
  }
}
</script>

<style scoped>
.login-page {
  height: 100vh; display: flex; align-items: center; justify-content: center;
  background: var(--dn-bg);
}
.login-card {
  width: 360px; padding: 36px 32px 28px;
  background: var(--dn-surface);
  border: 1px solid var(--dn-border);
  border-radius: 12px;
}
.login-logo { text-align: center; font-size: 34px; line-height: 1; margin-bottom: 12px; }
.login-title {
  text-align: center; font-size: 18px; font-weight: 600; color: var(--dn-text);
}
.login-sub {
  text-align: center; font-size: 12px; color: var(--dn-text-faint);
  margin: 6px 0 24px; letter-spacing: 1px;
}
.login-btn { width: 100%; letter-spacing: 6px; }
</style>
