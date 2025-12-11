"""UserInteraction model representing records of user interactions with the platform."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class UserInteractionBase(BaseModel):
    user_id: str
    interaction_type: str  # e.g., 'view', 'search', 'exercise', 'query', 'navigation'


class UserInteractionCreate(UserInteractionBase):
    content: Optional[str] = None
    result: Optional[str] = None
    session_id: Optional[str] = None
    chapter_id: Optional[str] = None


class UserInteractionUpdate(BaseModel):
    result: Optional[str] = None
    content: Optional[str] = None


class UserInteraction(UserInteractionBase):
    id: str
    chapter_id: Optional[str] = None
    content: Optional[str] = None
    result: Optional[str] = None
    timestamp: datetime
    session_id: Optional[str] = None

    class Config:
        from_attributes = True