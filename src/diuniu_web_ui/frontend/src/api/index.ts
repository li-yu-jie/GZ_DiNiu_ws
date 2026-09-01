import { http } from './http'

// ---------------- 鉴权 ----------------
export function login(username: string, password: string) {
  return http.post('/auth/login', { username, password })
}
export function changePassword(old_password: string, new_password: string) {
  return http.post('/auth/password', { old_password, new_password })
}

// ---------------- 账号管理（Admin） ----------------
export function listUsers() { return http.get('/users') }
export function createUser(username: string, password: string, role: string) {
  return http.post('/users', { username, password, role })
}
export function updateUserRole(id: number, role: string) {
  return http.put(`/users/${id}/role`, { role })
}
export function resetUserPassword(id: number, new_password: string) {
  return http.put(`/users/${id}/password`, { new_password })
}
export function deleteUser(id: number) { return http.delete(`/users/${id}`) }

// ---------------- 模式 / 状态 ----------------
export function setMode(mode: 'mapping' | 'navigation' | 'stop') {
  return http.post(`/mode/${mode}`)
}
export function getStatus() { return http.get('/status') }

// ---------------- 建图保存 ----------------
export function saveMapping(name?: string) {
  return http.post('/mapping/save', name ? { name } : {})
}
export function getMappingSaveStatus() { return http.get('/mapping/save/status') }

// ---------------- 多地图库 ----------------
export interface MapInfo {
  id: string
  name: string
  created_at: string
}
export function listMaps() { return http.get<{ maps: MapInfo[]; active: string }>('/maps') }
export function activateMap(id: string) { return http.post(`/maps/${id}/activate`) }
export function renameMap(id: string, name: string) { return http.put(`/maps/${id}`, { name }) }
export function deleteMap(id: string) { return http.delete(`/maps/${id}`) }

// ---------------- 航点 ----------------
export interface Waypoint {
  id: number
  name: string
  x: number
  y: number
  yaw: number
}
export function listWaypoints() { return http.get<Waypoint[]>('/waypoints') }
export function addWaypoint(name: string, x: number, y: number, yaw: number) {
  return http.post('/waypoints', { name, x, y, yaw })
}
export function updateWaypoint(id: number, fields: Partial<Omit<Waypoint, 'id'>>) {
  return http.put(`/waypoints/${id}`, fields)
}
export function deleteWaypoint(id: number) { return http.delete(`/waypoints/${id}`) }

// ---------------- 地图 / 禁区（图片二进制） ----------------
export async function getMapImage(): Promise<{ blob: Blob; meta: Record<string, string> }> {
  const resp = await http.get('/map/image', { responseType: 'blob' })
  return { blob: resp.data, meta: resp.headers as any }
}
export function saveMapImage(png: Blob) {
  return http.post('/map/save', png, { headers: { 'Content-Type': 'image/png' } })
}
export async function getKeepoutImage(): Promise<Blob> {
  const resp = await http.get('/keepout/image', { responseType: 'blob' })
  return resp.data
}
export function saveKeepoutImage(png: Blob) {
  return http.post('/keepout/save', png, { headers: { 'Content-Type': 'image/png' } })
}
