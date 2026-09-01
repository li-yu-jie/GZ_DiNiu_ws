# =============================================================================
# db.py — SQLite 用户库（users.db）
#
# 单文件数据库便于跟车备份；密码只存 bcrypt 哈希，绝不落地明文。
# 启动自检：若库为空，自动创建超级管理员 admin/admin（2026-09-02 用户指定），
# 可用环境变量 DIUNIU_ADMIN_PASSWORD 覆盖初始密码。
# =============================================================================
import os
import sqlite3
import threading
import time

import bcrypt

DATA_DIR = os.environ.get('DIUNIU_DATA_DIR', os.path.expanduser('~/.diuniu_web_ui'))
DB_PATH = os.path.join(DATA_DIR, 'users.db')

ROLES = ('admin', 'operator', 'viewer')
DEFAULT_ADMIN_USERNAME = 'admin'
# 初始管理员密码：环境变量优先，默认 'admin'（内网部署，登录后建议尽快修改）
DEFAULT_ADMIN_PASSWORD = os.environ.get('DIUNIU_ADMIN_PASSWORD', 'admin')

_lock = threading.Lock()


def _connect():
    os.makedirs(DATA_DIR, exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    """建表 + 空库时写入默认超级管理员。"""
    with _lock, _connect() as conn:
        conn.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                username TEXT UNIQUE NOT NULL,
                password_hash TEXT NOT NULL,
                role TEXT NOT NULL CHECK(role IN ('admin', 'operator', 'viewer')),
                created_at REAL NOT NULL
            )
        ''')
        n = conn.execute('SELECT COUNT(*) FROM users').fetchone()[0]
        if n == 0:
            conn.execute(
                'INSERT INTO users (username, password_hash, role, created_at) VALUES (?,?,?,?)',
                (DEFAULT_ADMIN_USERNAME, hash_password(DEFAULT_ADMIN_PASSWORD),
                 'admin', time.time()))
            print(f'[db] 用户库为空，已创建初始管理员 {DEFAULT_ADMIN_USERNAME}（默认密码登录后请尽快修改）')


def hash_password(plain: str) -> str:
    return bcrypt.hashpw(plain.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')


def verify_password(plain: str, hashed: str) -> bool:
    try:
        return bcrypt.checkpw(plain.encode('utf-8'), hashed.encode('utf-8'))
    except ValueError:
        return False


# ---------------- 查询 ----------------
def get_user_by_name(username: str):
    with _lock, _connect() as conn:
        row = conn.execute('SELECT * FROM users WHERE username = ?', (username,)).fetchone()
        return dict(row) if row else None


def get_user_by_id(user_id: int):
    with _lock, _connect() as conn:
        row = conn.execute('SELECT * FROM users WHERE id = ?', (user_id,)).fetchone()
        return dict(row) if row else None


def list_users():
    with _lock, _connect() as conn:
        rows = conn.execute(
            'SELECT id, username, role, created_at FROM users ORDER BY id').fetchall()
        return [dict(r) for r in rows]


# ---------------- 写操作（均不返回哈希） ----------------
def create_user(username: str, password: str, role: str):
    """返回 (user_dict, err)；用户名重复时 err 非空。"""
    if role not in ROLES:
        return None, f'非法角色: {role}'
    with _lock, _connect() as conn:
        try:
            cur = conn.execute(
                'INSERT INTO users (username, password_hash, role, created_at) VALUES (?,?,?,?)',
                (username, hash_password(password), role, time.time()))
        except sqlite3.IntegrityError:
            return None, f'用户名已存在: {username}'
        return {'id': cur.lastrowid, 'username': username, 'role': role}, None


def update_password(user_id: int, new_password: str) -> bool:
    with _lock, _connect() as conn:
        cur = conn.execute('UPDATE users SET password_hash = ? WHERE id = ?',
                           (hash_password(new_password), user_id))
        return cur.rowcount > 0


def update_role(user_id: int, role: str):
    """返回 (ok, err)。不允许把最后一个 admin 降级。"""
    if role not in ROLES:
        return False, f'非法角色: {role}'
    with _lock, _connect() as conn:
        user = conn.execute('SELECT * FROM users WHERE id = ?', (user_id,)).fetchone()
        if not user:
            return False, '用户不存在'
        if user['role'] == 'admin' and role != 'admin':
            n = conn.execute("SELECT COUNT(*) FROM users WHERE role = 'admin'").fetchone()[0]
            if n <= 1:
                return False, '系统至少保留一个管理员'
        conn.execute('UPDATE users SET role = ? WHERE id = ?', (role, user_id))
        return True, ''


def delete_user(user_id: int):
    """返回 (ok, err)。不允许删除最后一个 admin。"""
    with _lock, _connect() as conn:
        user = conn.execute('SELECT * FROM users WHERE id = ?', (user_id,)).fetchone()
        if not user:
            return False, '用户不存在'
        if user['role'] == 'admin':
            n = conn.execute("SELECT COUNT(*) FROM users WHERE role = 'admin'").fetchone()[0]
            if n <= 1:
                return False, '系统至少保留一个管理员'
        conn.execute('DELETE FROM users WHERE id = ?', (user_id,))
        return True, ''
