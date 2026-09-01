# =============================================================================
# auth.py — JWT 签发/校验 + FastAPI 角色守卫
#
# HS256 无状态令牌；密钥取环境变量 DIUNIU_JWT_SECRET，缺省时随机生成并落盘到
# 数据目录（重启不失效，换机部署各机密钥独立）。
# 角色按层级比较：admin > operator > viewer，require_role('operator') 表示
# "operator 及以上"。
# =============================================================================
import os
import secrets
import time

import jwt
from fastapi import Depends, HTTPException
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from . import db

TOKEN_TTL_SECONDS = int(os.environ.get('DIUNIU_TOKEN_TTL', str(12 * 3600)))  # 默认 12h

ROLE_LEVEL = {'viewer': 1, 'operator': 2, 'admin': 3}

_bearer = HTTPBearer(auto_error=False)


def _load_secret() -> str:
    env = os.environ.get('DIUNIU_JWT_SECRET')
    if env:
        return env
    path = os.path.join(db.DATA_DIR, 'jwt_secret')
    if os.path.exists(path):
        with open(path) as f:
            return f.read().strip()
    os.makedirs(db.DATA_DIR, exist_ok=True)
    secret = secrets.token_hex(32)
    with open(path, 'w') as f:
        f.write(secret)
    os.chmod(path, 0o600)
    return secret


SECRET = _load_secret()


def issue_token(user: dict) -> str:
    now = int(time.time())
    payload = {
        'sub': str(user['id']),
        'username': user['username'],
        'role': user['role'],
        'iat': now,
        'exp': now + TOKEN_TTL_SECONDS,
    }
    return jwt.encode(payload, SECRET, algorithm='HS256')


def decode_token(token: str):
    """返回 payload 或 None。"""
    try:
        return jwt.decode(token, SECRET, algorithms=['HS256'])
    except jwt.PyJWTError:
        return None


def get_current_user(cred: HTTPAuthorizationCredentials = Depends(_bearer)):
    """Depends：强校验 Bearer Token，返回用户 dict（含 id/username/role）。"""
    if cred is None:
        raise HTTPException(401, '未登录或缺少凭证')
    payload = decode_token(cred.credentials)
    if payload is None:
        raise HTTPException(401, '凭证无效或已过期')
    user = db.get_user_by_id(int(payload['sub']))
    if user is None:
        raise HTTPException(401, '账号不存在或已删除')
    # 角色可能被管理员中途调整，以数据库当前值为准
    return {'id': user['id'], 'username': user['username'], 'role': user['role']}


def require_role(min_role: str):
    """Depends 工厂：要求当前用户角色 >= min_role。"""
    min_level = ROLE_LEVEL[min_role]

    def checker(user: dict = Depends(get_current_user)):
        if ROLE_LEVEL.get(user['role'], 0) < min_level:
            raise HTTPException(403, f'权限不足，需要 {min_role} 及以上角色')
        return user

    return checker
