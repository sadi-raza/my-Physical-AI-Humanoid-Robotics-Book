"""RAGResponse model representing AI-generated answers based on textbook content with APA citations and confidence scores."""

from datetime import datetime
from typing import Optional, Dict, Any, List
from pydantic import BaseModel


class RAGResponseBase(BaseModel):
    query: str
    response: str
    confidence_score: float  # Between 0.0 and 1.0


class RAGResponseCreate(RAGResponseBase):
    citations: List[Dict[str, Any]]
    book_content_used: Optional[List[str]] = []


class RAGResponseUpdate(BaseModel):
    response: Optional[str] = None
    citations: Optional[List[Dict[str, Any]]] = None
    confidence_score: Optional[float] = None
    response_time_ms: Optional[int] = None


class RAGResponse(RAGResponseBase):
    id: str
    citations: List[Dict[str, Any]]
    response_time_ms: Optional[int] = None
    book_content_used: List[str] = []
    created_at: datetime

    class Config:
        from_attributes = True