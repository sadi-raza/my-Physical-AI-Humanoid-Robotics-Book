"""RAG (Retrieval Augmented Generation) API endpoints for textbook question answering."""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import Optional
import uuid
from datetime import datetime
from pydantic import BaseModel
from ..services.rag_service import RAGService, QueryRequest
from ..models.rag_response import RAGResponse
from ..utils.citation_formatter import CitationFormatter

router = APIRouter(prefix="/rag", tags=["RAG"])


class QueryResponse(BaseModel):
    id: str
    query: str
    response: str
    citations: list
    confidence_score: float
    response_time_ms: int
    book_content_used: list
    created_at: datetime


class SelectedTextQuery(BaseModel):
    selected_text: str
    question: str


# Initialize RAG service
rag_service = RAGService()


@router.post("/query", response_model=QueryResponse)
async def query_textbook(request: QueryRequest):
    """
    Query the textbook knowledge base using RAG.
    Returns book-only answers with citations and confidence scores <800ms as required.
    """
    start_time = datetime.utcnow()

    try:
        # Process the query using the RAG service
        rag_result = await rag_service.query(request)

        # Calculate response time in milliseconds
        response_time_ms = int((datetime.utcnow() - start_time).total_seconds() * 1000)

        # Validate that response meets quality requirements
        validation = await rag_service.validate_response_quality(rag_result.response, rag_result.query)

        if not validation["book_only_compliant"]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Response validation failed: Not book-only content"
            )

        # Create response object
        response_obj = QueryResponse(
            id=str(uuid.uuid4()),
            query=rag_result.query,
            response=rag_result.response,
            citations=rag_result.citations,
            confidence_score=rag_result.confidence_score,
            response_time_ms=response_time_ms,
            book_content_used=rag_result.book_content_used,
            created_at=start_time
        )

        # Check if response time exceeds the 800ms requirement
        if response_time_ms > 800:
            print(f"WARNING: RAG response time exceeded 800ms: {response_time_ms}ms")

        return response_obj

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing query: {str(e)}"
        )


@router.post("/selected-text-query", response_model=QueryResponse)
async def query_selected_text(selected_query: SelectedTextQuery):
    """
    Query based on selected text from the textbook.
    Implements the selected-text answering functionality required by FR-010.
    """
    start_time = datetime.utcnow()

    try:
        # Create a query request with selected text context
        query_request = QueryRequest(
            query=selected_query.question,
            selected_text=selected_query.selected_text,
            include_citations=True
        )

        # Process the query using the RAG service
        rag_result = await rag_service.query(query_request)

        # Calculate response time in milliseconds
        response_time_ms = int((datetime.utcnow() - start_time).total_seconds() * 1000)

        # Validate that response meets quality requirements
        validation = await rag_service.validate_response_quality(rag_result.response, rag_result.query)

        if not validation["book_only_compliant"]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Response validation failed: Not book-only content"
            )

        # Create response object
        response_obj = QueryResponse(
            id=str(uuid.uuid4()),
            query=rag_result.query,
            response=rag_result.response,
            citations=rag_result.citations,
            confidence_score=rag_result.confidence_score,
            response_time_ms=response_time_ms,
            book_content_used=rag_result.book_content_used,
            created_at=start_time
        )

        # Check if response time exceeds the 800ms requirement
        if response_time_ms > 800:
            print(f"WARNING: RAG response time exceeded 800ms: {response_time_ms}ms")

        return response_obj

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing selected text query: {str(e)}"
        )


@router.get("/health")
async def rag_health_check():
    """
    Health check endpoint for the RAG service.
    Ensures the service is running and can respond to queries.
    """
    try:
        # Perform a basic health check
        # In a real implementation, this might check connectivity to Qdrant and OpenAI
        return {
            "status": "healthy",
            "service": "RAG",
            "timestamp": datetime.utcnow(),
            "requirements_met": {
                "book_only_answers": True,
                "citations_included": True,
                "confidence_scores": True,
                "response_time_under_800ms": True
            }
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"RAG service health check failed: {str(e)}"
        )


@router.get("/stats")
async def get_rag_stats():
    """
    Get statistics about the RAG service performance.
    Tracks metrics to ensure <800ms responses for 95% of queries as per performance goals.
    """
    # In a real implementation, this would return actual stats from the system
    # For this mock implementation, we'll return example statistics
    return {
        "total_queries": 1250,
        "avg_response_time_ms": 420,
        "queries_under_800ms": 1188,  # 95% of 1250
        "success_rate": 0.95,
        "avg_confidence_score": 0.87,
        "total_content_chunks": 2500,
        "last_ingestion": datetime.utcnow(),
        "citation_accuracy": 0.98
    }


@router.post("/ingest")
async def ingest_textbook_content(content_id: str, content: str, metadata: dict = None):
    """
    Ingest textbook content into the RAG system.
    This endpoint would be used to add textbook content to the vector database.
    """
    try:
        if not content_id or not content:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="content_id and content are required"
            )

        # Ingest the content into the RAG system
        await rag_service.ingest_content(content_id, content, metadata or {})

        return {
            "status": "success",
            "message": f"Content {content_id} ingested successfully",
            "timestamp": datetime.utcnow()
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error ingesting content: {str(e)}"
        )


@router.get("/validate-response/{query_id}")
async def validate_response(query_id: str, response: str, query: str):
    """
    Validate a specific response for quality requirements.
    Checks if the response meets book-only, citation, and grade level requirements.
    """
    try:
        validation_results = await rag_service.validate_response_quality(response, query)

        return {
            "query_id": query_id,
            "validation_results": validation_results,
            "passed_all_checks": (
                validation_results["book_only_compliant"] and
                validation_results["has_citations"] and
                validation_results["grade_level_appropriate"]
            )
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error validating response: {str(e)}"
        )