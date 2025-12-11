"""Skill for ingesting textbook content into Qdrant vector database."""

from typing import Dict, Any
from ..services.content_ingestion import ContentIngestionPipeline
import asyncio


async def rag_ingest_skill(textbook_path: str) -> Dict[str, Any]:
    """
    Skill to ingest textbook content into the RAG system.

    Args:
        textbook_path: Path to the textbook content directory

    Returns:
        Dictionary with ingestion statistics
    """
    pipeline = ContentIngestionPipeline()
    return await pipeline.ingest_textbook_content(textbook_path)


def execute(textbook_path: str) -> Dict[str, Any]:
    """
    Execute the rag-ingest skill synchronously.

    Args:
        textbook_path: Path to the textbook content directory

    Returns:
        Dictionary with ingestion statistics
    """
    return asyncio.run(rag_ingest_skill(textbook_path))