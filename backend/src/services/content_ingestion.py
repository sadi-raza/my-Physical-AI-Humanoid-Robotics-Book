"""Content ingestion pipeline for textbook content into Qdrant vector database."""

import asyncio
import hashlib
from typing import List, Dict, Any, Optional
from pathlib import Path
import json
import logging
from ..config import settings
from ..services.rag_service import RAGService
from qdrant_client.http.models import PointStruct, VectorParams, Distance, Batch
from sentence_transformers import SentenceTransformer
import numpy as np
from datetime import datetime


class ContentIngestionPipeline:
    """Pipeline for ingesting textbook content into the RAG system."""

    def __init__(self):
        self.rag_service = RAGService()
        self.collection_name = settings.vector_collection_name
        self.embedding_model = SentenceTransformer(settings.vector_embedding_model)
        self.logger = logging.getLogger(__name__)

    async def ingest_textbook_content(self, textbook_path: str) -> Dict[str, Any]:
        """
        Ingest textbook content from a directory structure into the vector database.

        Args:
            textbook_path: Path to the textbook content directory

        Returns:
            Dictionary with ingestion statistics
        """
        self.logger.info(f"Starting ingestion of textbook content from {textbook_path}")

        stats = {
            "modules_processed": 0,
            "chapters_processed": 0,
            "sections_processed": 0,
            "total_chunks_ingested": 0,
            "start_time": datetime.utcnow().isoformat()
        }

        textbook_dir = Path(textbook_path)
        if not textbook_dir.exists():
            raise FileNotFoundError(f"Textbook directory does not exist: {textbook_path}")

        # Process each module directory
        for module_dir in textbook_dir.iterdir():
            if module_dir.is_dir():
                await self._process_module(module_dir, stats)

        stats["end_time"] = datetime.utcnow().isoformat()
        self.logger.info(f"Ingestion completed. Stats: {stats}")

        return stats

    async def _process_module(self, module_dir: Path, stats: Dict[str, Any]):
        """Process a single module directory."""
        self.logger.info(f"Processing module: {module_dir.name}")
        stats["modules_processed"] += 1

        # Process each chapter in the module
        for chapter_dir in module_dir.iterdir():
            if chapter_dir.is_dir():
                await self._process_chapter(chapter_dir, stats, module_dir.name)

    async def _process_chapter(self, chapter_dir: Path, stats: Dict[str, Any], module_name: str):
        """Process a single chapter directory."""
        self.logger.info(f"Processing chapter: {chapter_dir.name} in module: {module_name}")
        stats["chapters_processed"] += 1

        # Process the chapter's main content file
        for content_file in chapter_dir.glob("*.md"):
            await self._process_content_file(content_file, stats, module_name, chapter_dir.name)

        # Process any section files
        for section_file in chapter_dir.glob("sections/*.md"):
            await self._process_content_file(section_file, stats, module_name, chapter_dir.name, "section")

    async def _process_content_file(self, file_path: Path, stats: Dict[str, Any], module_name: str, chapter_name: str, content_type: str = "chapter"):
        """Process a single content file."""
        self.logger.info(f"Processing content file: {file_path}")

        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract content sections and create chunks
        chunks = self._create_content_chunks(content, file_path, module_name, chapter_name, content_type)

        # Ingest chunks into vector database
        await self._ingest_chunks(chunks)

        stats["total_chunks_ingested"] += len(chunks)

        if content_type == "section":
            stats["sections_processed"] += 1

    def _create_content_chunks(self, content: str, file_path: Path, module_name: str, chapter_name: str, content_type: str) -> List[Dict[str, Any]]:
        """
        Create content chunks for vector storage.

        Args:
            content: The content to chunk
            file_path: Path to the source file
            module_name: Name of the module
            chapter_name: Name of the chapter
            content_type: Type of content ('chapter', 'section', etc.)

        Returns:
            List of content chunks with metadata
        """
        # Split content into chunks of reasonable size (e.g., 512 tokens)
        # This is a simplified approach - in a real implementation, we'd use more sophisticated chunking
        chunk_size = 1000  # characters per chunk
        chunks = []

        # Split content into paragraphs first
        paragraphs = content.split('\n\n')

        current_chunk = ""
        chunk_id = 0

        for paragraph in paragraphs:
            # If adding this paragraph would exceed chunk size, start a new chunk
            if len(current_chunk) + len(paragraph) > chunk_size and current_chunk:
                # Create a chunk with the current content
                chunk_data = {
                    "id": f"{module_name}_{chapter_name}_{content_type}_{chunk_id}",
                    "content": current_chunk.strip(),
                    "module": module_name,
                    "chapter": chapter_name,
                    "type": content_type,
                    "source_file": str(file_path),
                    "created_at": datetime.utcnow().isoformat()
                }
                chunks.append(chunk_data)

                current_chunk = paragraph
                chunk_id += 1
            else:
                current_chunk += "\n\n" + paragraph

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunk_data = {
                "id": f"{module_name}_{chapter_name}_{content_type}_{chunk_id}",
                "content": current_chunk.strip(),
                "module": module_name,
                "chapter": chapter_name,
                "type": content_type,
                "source_file": str(file_path),
                "created_at": datetime.utcnow().isoformat()
            }
            chunks.append(chunk_data)

        return chunks

    async def _ingest_chunks(self, chunks: List[Dict[str, Any]]):
        """Ingest content chunks into the vector database."""
        if not chunks:
            return

        # Generate embeddings for all chunks
        contents = [chunk["content"] for chunk in chunks]
        embeddings = self.embedding_model.encode(contents)

        # Prepare points for Qdrant
        points = []
        for i, chunk in enumerate(chunks):
            # Create a unique ID for the chunk
            content_hash = hashlib.md5(chunk["content"].encode('utf-8')).hexdigest()
            point_id = f"{chunk['id']}_{content_hash[:8]}"

            # Prepare metadata
            payload = {
                "content": chunk["content"],
                "module": chunk["module"],
                "chapter": chunk["chapter"],
                "type": chunk["type"],
                "source_file": chunk["source_file"],
                "created_at": chunk["created_at"]
            }

            # Create point
            point = PointStruct(
                id=point_id,
                vector=embeddings[i].tolist(),  # Convert numpy array to list
                payload=payload
            )
            points.append(point)

        # Upload points to Qdrant in batches
        batch_size = 100  # Adjust based on your needs
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.rag_service.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            self.logger.info(f"Ingested batch of {len(batch)} points")

    async def ingest_single_content(self, content_id: str, content: str, metadata: Dict[str, Any]) -> str:
        """
        Ingest a single piece of content into the vector database.

        Args:
            content_id: Unique identifier for the content
            content: Content to ingest
            metadata: Additional metadata about the content

        Returns:
            ID of the ingested content
        """
        # Generate embedding for the content
        embedding = self.embedding_model.encode([content])[0].tolist()

        # Create content hash for unique ID
        content_hash = hashlib.md5(content.encode('utf-8')).hexdigest()
        point_id = f"{content_id}_{content_hash[:8]}"

        # Prepare payload with metadata
        payload = {
            "content": content,
            **metadata,
            "created_at": datetime.utcnow().isoformat()
        }

        # Create point
        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload=payload
        )

        # Upload to Qdrant
        self.rag_service.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )

        self.logger.info(f"Ingested single content with ID: {point_id}")
        return point_id

    async def update_content(self, content_id: str, new_content: str, metadata: Dict[str, Any]) -> str:
        """
        Update existing content in the vector database.

        Args:
            content_id: ID of the content to update
            new_content: New content to replace the old one
            metadata: Updated metadata

        Returns:
            ID of the updated content
        """
        # First, delete the old content
        try:
            # Find points with the content_id in their ID
            search_result = self.rag_service.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=None,  # We'll filter by ID pattern
                limit=1000
            )

            # Filter points that match our content_id pattern
            points_to_delete = []
            for point in search_result[0]:  # search_result[0] contains the points
                if content_id in point.id:
                    points_to_delete.append(point.id)

            if points_to_delete:
                self.rag_service.qdrant_client.delete(
                    collection_name=self.collection_name,
                    points_selector=points_to_delete
                )
                self.logger.info(f"Deleted {len(points_to_delete)} points for content ID: {content_id}")
        except Exception as e:
            self.logger.warning(f"Could not delete old content for ID {content_id}: {e}")

        # Then, add the new content
        return await self.ingest_single_content(content_id, new_content, metadata)

    async def delete_content(self, content_id: str) -> bool:
        """
        Delete content from the vector database.

        Args:
            content_id: ID of the content to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            # Find points with the content_id in their ID
            search_result = self.rag_service.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=None,
                limit=1000
            )

            # Filter points that match our content_id pattern
            points_to_delete = []
            for point in search_result[0]:  # search_result[0] contains the points
                if content_id in point.id:
                    points_to_delete.append(point.id)

            if points_to_delete:
                self.rag_service.qdrant_client.delete(
                    collection_name=self.collection_name,
                    points_selector=points_to_delete
                )
                self.logger.info(f"Deleted {len(points_to_delete)} points for content ID: {content_id}")
                return True
            else:
                self.logger.warning(f"No points found for content ID: {content_id}")
                return False
        except Exception as e:
            self.logger.error(f"Error deleting content with ID {content_id}: {e}")
            return False


# Example usage function
async def run_ingestion_pipeline():
    """Example of how to use the ingestion pipeline."""
    pipeline = ContentIngestionPipeline()

    # Ingest textbook content from the frontend docs directory
    textbook_path = "../../../frontend/docs"  # Relative to backend/src directory

    try:
        stats = await pipeline.ingest_textbook_content(textbook_path)
        print(f"Ingestion completed successfully: {stats}")
    except Exception as e:
        print(f"Error during ingestion: {e}")


if __name__ == "__main__":
    # Run the ingestion pipeline if this script is executed directly
    asyncio.run(run_ingestion_pipeline())