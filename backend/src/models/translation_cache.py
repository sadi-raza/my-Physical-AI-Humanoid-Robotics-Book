"""TranslationCache model representing cached Urdu translations of textbook content."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class TranslationCacheBase(BaseModel):
    content_id: str
    content_type: str  # 'chapter' or 'section'
    original_content_hash: str


class TranslationCacheCreate(TranslationCacheBase):
    urdu_translation: str


class TranslationCacheUpdate(BaseModel):
    urdu_translation: Optional[str] = None
    original_content_hash: Optional[str] = None


class TranslationCache(TranslationCacheBase):
    id: str
    urdu_translation: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True