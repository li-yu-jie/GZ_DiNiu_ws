# =============================================================================
# routes_auth.py — 登录 / 当前用户 / 修改自己的密码
# =============================================================================
import threading
import time

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from . import db
from .auth import get_current_user, issue_token

router = APIRouter(prefix='/api/auth', tags=['auth'])

# 登录限流：同一 IP 连续失败 5 次锁定 60 秒（bcrypt 本身慢是天然减速，
# 但挡不住分布式试口令；内存计数够用，重启清零可接受）
_FAIL_WINDOW_S = 300
_FAIL_LIMIT = 5
_LOCKOUT_S = 60
_login_fails = {}          # ip -> [失败时间戳...]
_login_lock = threading.Lock()


def _check_rate_limit(ip: str):
    now = time.time()
    with _login_lock:
        fails = [t for t in _login_fails.get(ip, []) if now - t < _FAIL_WINDOW_S]
        _login_fails[ip] = fails
        if len(fails) >= _FAIL_LIMIT and now - fails[-1] < _LOCKOUT_S:
            raise HTTPException(429, '失败次数过多，请 60 秒后再试')


def _record_fail(ip: str):
    with _login_lock:
        _login_fails.setdefault(ip, []).append(time.time())


def _clear_fails(ip: str):
    with _login_lock:
        _login_fails.pop(ip, None)


class LoginIn(BaseModel):
    username: str
    password: str


class PasswordChange(BaseModel):
    old_password: str
    new_password: str


@router.post('/login')
def login(body: LoginIn, request: Request):
    ip = request.client.host if request.client else 'unknown'
    _check_rate_limit(ip)
    user = db.get_user_by_name(body.username)
    if user is None or not db.verify_password(body.password, user['password_hash']):
        _record_fail(ip)
        raise HTTPException(401, '用户名或密码错误')
    _clear_fails(ip)
    return {
        'token': issue_token(user),
        'user': {'id': user['id'], 'username': user['username'], 'role': user['role']},
    }


@router.get('/me')
def me(user: dict = Depends(get_current_user)):
    return user


@router.post('/password')
def change_password(body: PasswordChange, user: dict = Depends(get_current_user)):
    row = db.get_user_by_id(user['id'])
    if not db.verify_password(body.old_password, row['password_hash']):
        raise HTTPException(400, '原密码错误')
    if len(body.new_password) < 6:
        raise HTTPException(400, '新密码至少 6 位')
    db.update_password(user['id'], body.new_password)
    return {'ok': True}
