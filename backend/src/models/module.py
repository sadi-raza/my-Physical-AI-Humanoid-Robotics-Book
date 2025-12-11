"""Module model representing one of the 5 core modules (ROS2, Gazebo/Unity, Isaac, VLA, Capstone)."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class ModuleBase(BaseModel):
    name: str
    title: str
    description: Optional[str] = None
    order: int


class ModuleCreate(ModuleBase):
    pass


class ModuleUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    order: Optional[int] = None


class Module(ModuleBase):
    id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True