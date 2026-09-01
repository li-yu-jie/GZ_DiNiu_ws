import { defineStore } from 'pinia'
import { http } from '../api/http'

export interface UserInfo {
  id: number
  username: string
  role: string
}

const ROLE_LEVEL: Record<string, number> = { viewer: 1, operator: 2, admin: 3 }

export const useAuthStore = defineStore('auth', {
  state: () => ({
    token: localStorage.getItem('diuniu_token') || '',
    user: JSON.parse(localStorage.getItem('diuniu_user') || 'null') as UserInfo | null,
  }),
  getters: {
    isLoggedIn: (s) => !!s.token,
    role: (s) => s.user?.role || 'viewer',
    isAdmin(): boolean { return this.role === 'admin' },
    canOperate(): boolean { return (ROLE_LEVEL[this.role] || 0) >= ROLE_LEVEL.operator },
  },
  actions: {
    setSession(token: string, user: UserInfo) {
      this.token = token
      this.user = user
      localStorage.setItem('diuniu_token', token)
      localStorage.setItem('diuniu_user', JSON.stringify(user))
    },
    logout() {
      this.token = ''
      this.user = null
      localStorage.removeItem('diuniu_token')
      localStorage.removeItem('diuniu_user')
    },
    hasRole(minRole: string): boolean {
      return (ROLE_LEVEL[this.role] || 0) >= (ROLE_LEVEL[minRole] || 99)
    },
    async fetchMe(): Promise<boolean> {
      try {
        const { data } = await http.get('/auth/me')
        this.user = data
        localStorage.setItem('diuniu_user', JSON.stringify(data))
        return true
      } catch {
        this.logout()
        return false
      }
    },
  },
})

