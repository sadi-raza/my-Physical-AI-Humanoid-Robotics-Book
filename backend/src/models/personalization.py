"""PersonalizationProfile model representing user background information that influences content presentation."""

from datetime import datetime
from typing import Optional, Dict, Any, List
from pydantic import BaseModel


class PersonalizationProfileBase(BaseModel):
    user_id: str
    module_id: str


class PersonalizationProfileCreate(PersonalizationProfileBase):
    personalization_settings: Optional[Dict[str, Any]] = {}
    learning_progress: Optional[Dict[str, Any]] = {}
    preference_tags: Optional[List[str]] = []


class PersonalizationProfileUpdate(BaseModel):
    personalization_settings: Optional[Dict[str, Any]] = None
    learning_progress: Optional[Dict[str, Any]] = None
    preference_tags: Optional[List[str]] = None


class PersonalizationProfile(PersonalizationProfileBase):
    id: str
    personalization_settings: Dict[str, Any] = {}
    learning_progress: Dict[str, Any] = {}
    preference_tags: List[str] = []
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True