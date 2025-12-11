"""RAG (Retrieval Augmented Generation) service for textbook content retrieval."""

import asyncio
import time
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from ..models.rag_response import RAGResponseCreate
from ..utils.citation_formatter import CitationFormatter
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance, SearchRequest
from openai import AsyncOpenAI
from ..utils.content_validator import ContentValidator
from ..config import settings


class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None
    module_filter: Optional[str] = None
    include_citations: bool = True
    max_results: int = 5


class RAGService:
    """Implementation of RAG service that provides book-only answers with citations and confidence scores."""

    def __init__(self):
        # Initialize Qdrant client for vector storage
        if settings.qdrant_api_key:
            self.qdrant_client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10
            )
        else:
            # For local development without API key
            self.qdrant_client = QdrantClient(
                host=settings.qdrant_host,
                port=settings.qdrant_port
            )

        # Initialize OpenAI client for content generation
        self.openai_client = AsyncOpenAI(
            api_key=settings.openai_api_key
        )

        # Collection name for textbook content
        self.collection_name = settings.vector_collection_name

        # Initialize citation formatter
        self.citation_formatter = CitationFormatter()

        # Create collection if it doesn't exist
        try:
            self.qdrant_client.get_collection(self.collection_name)
        except:
            # Create collection with vector configuration
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=1536, distance=Distance.COSINE)  # Assuming OpenAI embeddings
            )

    async def query(self, query_request: QueryRequest) -> RAGResponseCreate:
        """
        Process a query and return a response with citations and confidence score.

        Args:
            query_request: Query parameters including the question and options

        Returns:
            RAGResponseCreate with response, citations, and confidence score
        """
        start_time = time.time()

        # Search for relevant content in the vector database
        search_results = await self.search_content(query_request.query, query_request.max_results)

        if not search_results:
            # If no relevant content found, return a response indicating this
            return RAGResponseCreate(
                query=query_request.query,
                response="I couldn't find relevant content in the textbook to answer your question.",
                citations=[],
                confidence_score=0.1,
                book_content_used=[]
            )

        # Prepare context from search results
        context = self._prepare_context(search_results)

        # Generate response using OpenAI
        response = await self._generate_response(query_request.query, context, query_request.selected_text)

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Format citations
        citations = self._format_citations(search_results)

        # Calculate confidence based on search result relevance
        confidence_score = self._calculate_confidence(search_results)

        # Create response object
        rag_response = RAGResponseCreate(
            query=query_request.query,
            response=response,
            citations=citations,
            confidence_score=confidence_score,
            book_content_used=[result['id'] for result in search_results]
        )

        return rag_response

    async def search_content(self, query: str, max_results: int = 5) -> List[Dict[str, Any]]:
        """
        Search for relevant content in the textbook using vector similarity.

        Args:
            query: The query string to search for
            max_results: Maximum number of results to return

        Returns:
            List of relevant content with metadata
        """
        try:
            # In a real implementation, we would generate embeddings for the query
            # For this mock, we'll return some dummy results
            # This is a simplified version - in reality, we'd use embeddings

            # Mock search results based on keyword matching (in real implementation, use vector search)
            mock_results = [
                {
                    "id": f"mock-chapter-{i}",
                    "content": f"This is mock content related to '{query}' for demonstration purposes.",
                    "source": f"ROS2 Module, Chapter {i}",
                    "page_reference": f"ros2/chapter-{i}",
                    "confidence": 0.85 + (0.15 / (i + 1))  # Higher confidence for first results
                }
                for i in range(1, max_results + 1)
            ]

            return mock_results
        except Exception as e:
            print(f"Error searching content: {e}")
            return []

    def _prepare_context(self, search_results: List[Dict[str, Any]]) -> str:
        """
        Prepare context from search results for the LLM.

        Args:
            search_results: List of search results

        Returns:
            Formatted context string
        """
        context_parts = []
        for result in search_results:
            context_parts.append(f"Source: {result['source']}\nContent: {result['content']}\n")

        return "\n".join(context_parts)

    async def _generate_response(self, query: str, context: str, selected_text: Optional[str] = None) -> str:
        """
        Generate a response using OpenAI based on the query and context.

        Args:
            query: The original query
            context: Retrieved context from the textbook
            selected_text: Selected text for context (if any)

        Returns:
            Generated response string
        """
        try:
            # Build the prompt for the LLM
            prompt = f"""
            You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.
            Use only the following context to answer the question. Do not use any external knowledge.

            Context:
            {context}

            Question: {query}
            """

            if selected_text:
                prompt += f"\n\nAdditional context from selected text: {selected_text}"

            # Call OpenAI API
            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are an assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided textbook content. Be precise, cite sources, and ensure responses are accurate."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )

            return response.choices[0].message.content
        except Exception as e:
            print(f"Error generating response: {e}")
            return f"I encountered an error processing your query: {str(e)}. Please try again."

    def _format_citations(self, search_results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Format citations from search results in APA style.

        Args:
            search_results: List of search results

        Returns:
            List of formatted citations
        """
        citations = []
        for result in search_results:
            citation = {
                "source": result.get('source', 'Unknown'),
                "text": result.get('content', '')[:200] + "...",  # First 200 chars
                "confidence": result.get('confidence', 0.0),
                "page_reference": result.get('page_reference', '')
            }
            citations.append(citation)

        return citations

    def _calculate_confidence(self, search_results: List[Dict[str, Any]]) -> float:
        """
        Calculate an overall confidence score based on search results.

        Args:
            search_results: List of search results

        Returns:
            Overall confidence score between 0.0 and 1.0
        """
        if not search_results:
            return 0.1

        # Average the confidence scores from search results
        total_confidence = sum(result.get('confidence', 0.0) for result in search_results)
        avg_confidence = total_confidence / len(search_results)

        # Adjust based on number of results (more relevant results = higher confidence)
        num_results_factor = min(len(search_results) / 3.0, 1.0)  # Cap at 1.0

        # Combine average confidence with number of results factor
        combined_confidence = (avg_confidence * 0.7) + (num_results_factor * 0.3)

        # Ensure it stays within bounds
        return max(0.1, min(1.0, combined_confidence))

    async def ingest_content(self, content_id: str, content: str, metadata: Dict[str, Any]):
        """
        Ingest textbook content into the vector database.

        Args:
            content_id: Unique identifier for the content
            content: The content to ingest
            metadata: Additional metadata about the content
        """
        try:
            # In a real implementation, we would:
            # 1. Generate embeddings for the content using OpenAI or another model
            # 2. Store the embeddings in Qdrant with associated metadata

            # For this mock implementation, we'll just simulate the ingestion
            print(f"Ingesting content {content_id} into RAG system")

            # In a real system, this would involve:
            # - Chunking the content appropriately
            # - Generating embeddings for each chunk
            # - Storing in the vector database with metadata
            pass
        except Exception as e:
            print(f"Error ingesting content: {e}")
            raise

    async def validate_response_quality(self, response: str, query: str) -> Dict[str, Any]:
        """
        Validate that the response meets quality requirements.

        Args:
            response: The generated response
            query: The original query

        Returns:
            Validation results
        """
        # Check if response is book-only (this is a simplified check)
        book_only_indicators = [
            "according to the textbook",
            "the textbook states",
            "based on the provided content",
            "as mentioned in",
            "from chapter",
            "in the module"
        ]

        book_only_compliant = any(indicator in response.lower() for indicator in book_only_indicators)

        # Check if response contains citations (this is simplified)
        has_citations = any(char in response for char in ["(", "[", "reference", "source"])

        # Validate grade level appropriateness
        grade_validation = ContentValidator.validate_grade_level(response)

        return {
            "book_only_compliant": book_only_compliant,
            "has_citations": has_citations,
            "grade_level_appropriate": grade_validation["is_valid"],
            "reading_ease": grade_validation["reading_ease"],
            "overall_quality_score": (
                (1 if book_only_compliant else 0) +
                (1 if has_citations else 0) +
                (1 if grade_validation["is_valid"] else 0)
            ) / 3.0
        }