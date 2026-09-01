import axios from 'axios'
import router from '../router'
import { rosBridge } from '../ros/bridge'
import { useAuthStore } from '../stores/auth'

export const http = axios.create({ baseURL: '/api', timeout: 15000 })

// 请求自动携带 Bearer Token
http.interceptors.request.use((config) => {
  const auth = useAuthStore()
  if (auth.token) {
    config.headers.Authorization = `Bearer ${auth.token}`
  }
  return config
})

// 401 → 清会话回登录页（后端令牌失效/被删号等情况）
http.interceptors.response.use(
  (resp) => resp,
  (error) => {
    if (error?.response?.status === 401) {
      const auth = useAuthStore()
      auth.logout()
      rosBridge.disconnect()
      if (router.currentRoute.value.path !== '/login') {
        router.push('/login')
      }
    }
    return Promise.reject(error)
  },
)

