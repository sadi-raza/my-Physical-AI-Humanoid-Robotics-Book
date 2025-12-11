"""Section model representing a section within a chapter (3-7 per chapter)."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel


class SectionBase(BaseModel):
    chapter_id: str
    title: str
    order: int
    content: Optional[str] = None


class SectionCreate(SectionBase):
    title: str  # Required for creation
    content: str  # Required for creation


class SectionUpdate(BaseModel):
    title: Optional[str] = None
    order: Optional[str] = None
    content: Optional[str] = None


class Section(SectionBase):
    id: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True