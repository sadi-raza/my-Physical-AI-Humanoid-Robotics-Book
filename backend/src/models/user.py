"""User model representing a researcher who accesses the textbook."""

from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, EmailStr


class UserBase(BaseModel):
    email: EmailStr
    name: str


class UserCreate(UserBase):
    password: str  # In a real implementation, this would be hashed
    background: Optional[str] = None
    research_interests: Optional[List[str]] = []


class UserUpdate(BaseModel):
    name: Optional[str] = None
    background: Optional[str] = None
    research_interests: Optional[List[str]] = None


class User(UserBase):
    id: str
    background: Optional[str] = None
    research_interests: Optional[List[str]] = []
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True