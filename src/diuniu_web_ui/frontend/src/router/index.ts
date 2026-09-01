import { createRouter, createWebHistory } from 'vue-router'
import { useAuthStore } from '../stores/auth'

const router = createRouter({
  history: createWebHistory(),
  routes: [
    { path: '/login', name: 'login', component: () => import('../views/LoginView.vue') },
    {
      path: '/',
      name: 'monitor',
      component: () => import('../views/MonitorView.vue'),
      meta: { roles: ['viewer', 'operator', 'admin'] },
    },
    {
      path: '/mapping',
      name: 'mapping',
      component: () => import('../views/MappingView.vue'),
      meta: { roles: ['admin'] },
    },
    {
      path: '/mapedit',
      name: 'mapedit',
      component: () => import('../views/MapEditView.vue'),
      meta: { roles: ['admin'] },
    },
    {
      path: '/users',
      name: 'users',
      component: () => import('../views/UsersView.vue'),
      meta: { roles: ['admin'] },
    },
    { path: '/:pathMatch(.*)*', redirect: '/' },
  ],
})

// 第一道防线：无合法 JWT 一律锁在登录页；角色不足的页面强制回主页
router.beforeEach((to) => {
  const auth = useAuthStore()
  if (to.path === '/login') {
    return auth.isLoggedIn ? '/' : true
  }
  if (!auth.isLoggedIn) {
    return { path: '/login', query: { redirect: to.fullPath } }
  }
  const roles = to.meta.roles as string[] | undefined
  if (roles && !roles.includes(auth.role)) {
    return '/'
  }
  return true
})

export default router
