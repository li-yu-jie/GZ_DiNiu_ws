# =============================================================================
# routes_users.py — 账号管理（仅 Admin）
# =============================================================================
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from . import db
from .auth import require_role

router = APIRouter(prefix='/api/users', tags=['users'])

_admin = Depends(require_role('admin'))


class UserCreate(BaseModel):
    username: str
    password: str
    role: str = 'viewer'


class RoleUpdate(BaseModel):
    role: str


class PasswordReset(BaseModel):
    new_password: str


@router.get('')
def list_users(_user: dict = _admin):
    return db.list_users()


@router.post('', status_code=201)
def create_user(body: UserCreate, _user: dict = _admin):
    if not body.username.strip():
        raise HTTPException(400, '用户名不能为空')
    if len(body.password) < 6:
        raise HTTPException(400, '密码至少 6 位')
    user, err = db.create_user(body.username.strip(), body.password, body.role)
    if err:
        raise HTTPException(400, err)
    return user


@router.put('/{user_id}/role')
def set_role(user_id: int, body: RoleUpdate, _user: dict = _admin):
    ok, err = db.update_role(user_id, body.role)
    if not ok:
        raise HTTPException(400, err)
    return {'ok': True}


@router.put('/{user_id}/password')
def reset_password(user_id: int, body: PasswordReset, _user: dict = _admin):
    if len(body.new_password) < 6:
        raise HTTPException(400, '密码至少 6 位')
    if not db.update_password(user_id, body.new_password):
        raise HTTPException(404, '用户不存在')
    return {'ok': True}


@router.delete('/{user_id}')
def remove_user(user_id: int, user: dict = _admin):
    if user_id == user['id']:
        raise HTTPException(400, '不能删除当前登录账号')
    ok, err = db.delete_user(user_id)
    if not ok:
        raise HTTPException(400, err)
    return {'ok': True}
