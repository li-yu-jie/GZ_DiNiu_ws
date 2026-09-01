// =============================================================================
// bridge.ts — rosbridge WebSocket 连接管理（第三道防线）
//
// 仅在登录成功后由应用显式调用 connect()；登出/401 时 disconnect()。
// 连接走 FastAPI /ws/rosbridge 鉴权代理（同源 8000 端口），rosbridge 本体
// 只监听 127.0.0.1，不对外开放——未登录连代理都建不起来。
// =============================================================================
import ROSLIB from './roslib-global'
import { reactive } from 'vue'

class RosBridge {
  ros: any = null
  state = reactive({ connected: false, connecting: false, error: '' })
  private url = ''

  connect(url?: string) {
    if (this.ros) return
    const token = localStorage.getItem('diuniu_token') || ''
    const proto = location.protocol === 'https:' ? 'wss' : 'ws'
    this.url = url || `${proto}://${location.host}/ws/rosbridge?token=${encodeURIComponent(token)}`
    this.state.connecting = true
    this.state.error = ''
    const ros = new ROSLIB.Ros({ url: this.url })
    ros.on('connection', () => {
      this.state.connected = true
      this.state.connecting = false
    })
    ros.on('error', () => {
      this.state.error = 'rosbridge 连接失败'
      this.state.connecting = false
    })
    ros.on('close', () => {
      this.state.connected = false
      this.state.connecting = false
      this.ros = null
      // 登录态下断线自动重连（token 过期时代理会 4401 拒绝，
      // REST 侧 401 处理器会清登录态，shouldReconnect 随即失效）
      setTimeout(() => {
        if (this.shouldReconnect) this.connect()
      }, 2000)
    })
    this.ros = ros
  }

  get shouldReconnect(): boolean {
    return !!localStorage.getItem('diuniu_token')
  }

  disconnect() {
    if (this.ros) {
      const ros = this.ros
      this.ros = null
      ros.close()
    }
    this.state.connected = false
    this.state.connecting = false
  }
}

export const rosBridge = new RosBridge()
