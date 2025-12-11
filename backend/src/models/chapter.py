"""Chapter model representing a chapter within a module containing educational content."""

from datetime import datetime
from typing import Optional, Dict, Any, List
from pydantic import BaseModel


class ChapterBase(BaseModel):
    module_id: str
    title: str
    order: int
    outcomes: Optional[str] = None
    content: Optional[str] = None


class ChapterCreate(ChapterBase):
    outcomes: str  # Required for creation
    content: str   # Required for creation


class ChapterUpdate(BaseModel):
    title: Optional[str] = None
    order: Optional[int] = None
    outcomes: Optional[str] = None
    content: Optional[str] = None
    exercises: Optional[List[Dict[str, Any]]] = None
    code_examples: Optional[Dict[str, Any]] = None
    diagrams: Optional[Dict[str, Any]] = None


class Chapter(ChapterBase):
    id: str
    exercises: Optional[List[Dict[str, Any]]] = []
    code_examples: Optional[Dict[str, Any]] = {}
    diagrams: Optional[Dict[str, Any]] = {}
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True