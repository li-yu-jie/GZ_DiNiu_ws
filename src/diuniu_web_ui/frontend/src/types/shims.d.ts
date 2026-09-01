// roslib / nipplejs 无官方类型，按 any 处理（ROS 消息结构由 rosbridge 动态决定）
declare module 'roslib'
declare module 'nipplejs'

declare module '*.vue' {
  import type { DefineComponent } from 'vue'
  const component: DefineComponent<{}, {}, any>
  export default component
}

