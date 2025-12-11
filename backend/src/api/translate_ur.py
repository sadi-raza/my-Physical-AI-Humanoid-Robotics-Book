"""Urdu Translation API endpoints for translating textbook content."""

from fastapi import APIRouter, HTTPException, status
from typing import Optional, Dict, Any
from pydantic import BaseModel
from datetime import datetime
import uuid
from ..services.translation_service import TranslationService

router = APIRouter(prefix="/translate-ur", tags=["translation"])

# Initialize translation service
translation_service = TranslationService()


class TranslationRequest(BaseModel):
    content_id: str
    content_type: str  # 'chapter', 'section', 'text', 'exercise', etc.
    content: str
    preserve_formatting: bool = True


class TranslationResponse(BaseModel):
    id: str
    content_id: str
    content_type: str
    original_content: str
    urdu_translation: str
    original_content_hash: str
    created_at: datetime


class TranslationQuery(BaseModel):
    content_id: str
    content_type: str
    query: str


@router.post("/", response_model=TranslationResponse)
async def translate_to_urdu(request: TranslationRequest):
    """
    Translate content to Urdu.
    Implements FR-007: System MUST provide Urdu translation functionality accessible via /api/translate-ur.
    """
    try:
        # Translate the content
        urdu_translation = await translation_service.translate_content(
            content_id=request.content_id,
            content_type=request.content_type,
            content=request.content,
            preserve_formatting=request.preserve_formatting
        )

        # Create response object
        response = TranslationResponse(
            id=str(uuid.uuid4()),
            content_id=request.content_id,
            content_type=request.content_type,
            original_content=request.content,
            urdu_translation=urdu_translation,
            original_content_hash=hash(request.content),  # Simplified hash
            created_at=datetime.utcnow()
        )

        return response

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error translating content to Urdu: {str(e)}"
        )


@router.post("/translate-chapter")
async def translate_chapter(chapter_data: Dict[str, Any]):
    """
    Translate an entire chapter to Urdu.
    Preserves diagrams, code examples, and other non-textual elements as required.
    """
    try:
        # Validate that chapter data has required fields
        required_fields = ["id", "title", "content"]
        for field in required_fields:
            if field not in chapter_data:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Missing required field: {field}"
                )

        # Create a Chapter-like object (in a real implementation, we'd use the actual model)
        from ..models.chapter import Chapter

        chapter = Chapter(
            id=chapter_data["id"],
            module_id=chapter_data.get("module_id", ""),
            title=chapter_data["title"],
            order=chapter_data.get("order", 1),
            outcomes=chapter_data.get("outcomes", ""),
            content=chapter_data.get("content", ""),
            exercises=chapter_data.get("exercises", []),
            code_examples=chapter_data.get("code_examples", {}),
            diagrams=chapter_data.get("diagrams", {}),
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )

        # Translate the chapter
        translated_chapter = await translation_service.translate_chapter(chapter)

        return {
            "status": "success",
            "translated_chapter": translated_chapter,
            "timestamp": datetime.utcnow()
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error translating chapter to Urdu: {str(e)}"
        )


@router.post("/translate-section")
async def translate_section(section_data: Dict[str, Any]):
    """
    Translate a section to Urdu.
    """
    try:
        # Validate that section data has required fields
        required_fields = ["id", "chapter_id", "title", "content"]
        for field in required_fields:
            if field not in section_data:
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail=f"Missing required field: {field}"
                )

        # Create a Section-like object (in a real implementation, we'd use the actual model)
        from ..models.section import Section

        section = Section(
            id=section_data["id"],
            chapter_id=section_data["chapter_id"],
            title=section_data["title"],
            order=section_data.get("order", 1),
            content=section_data["content"],
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )

        # Translate the section
        translated_section = await translation_service.translate_section(section)

        return {
            "status": "success",
            "translated_section": translated_section,
            "timestamp": datetime.utcnow()
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error translating section to Urdu: {str(e)}"
        )


@router.post("/query")
async def translate_query(query_data: TranslationQuery):
    """
    Translate a specific query or text to Urdu.
    """
    try:
        # Translate the query text
        urdu_translation = await translation_service.translate_content(
            content_id=query_data.content_id,
            content_type=query_data.content_type,
            content=query_data.query,
            preserve_formatting=True
        )

        return {
            "original_query": query_data.query,
            "urdu_translation": urdu_translation,
            "content_id": query_data.content_id,
            "content_type": query_data.content_type,
            "timestamp": datetime.utcnow()
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error translating query to Urdu: {str(e)}"
        )


@router.get("/health")
async def translation_health_check():
    """
    Health check for the Urdu translation service.
    """
    return {
        "status": "healthy",
        "service": "Urdu Translation API",
        "timestamp": datetime.utcnow(),
        "features": {
            "urdu_translation": True,
            "formatting_preservation": True,
            "technical_accuracy": True,
            "caching_mechanism": True
        }
    }


@router.get("/supported-content-types")
async def get_supported_content_types():
    """
    Get list of supported content types for translation.
    """
    return {
        "supported_types": [
            "text",
            "chapter",
            "section",
            "exercise",
            "code",
            "diagram_description",
            "lesson_content"
        ],
        "preserved_elements": [
            "code_examples",
            "diagrams",
            "mathematical_formulas",
            "technical_terms",
            "programming_syntax"
        ]
    }


@router.post("/batch-translate")
async def batch_translate(contents: list[TranslationRequest]):
    """
    Translate multiple content items to Urdu in a batch operation.
    """
    try:
        results = []
        for request in contents:
            urdu_translation = await translation_service.translate_content(
                content_id=request.content_id,
                content_type=request.content_type,
                content=request.content,
                preserve_formatting=request.preserve_formatting
            )

            result = TranslationResponse(
                id=str(uuid.uuid4()),
                content_id=request.content_id,
                content_type=request.content_type,
                original_content=request.content,
                urdu_translation=urdu_translation,
                original_content_hash=hash(request.content),  # Simplified hash
                created_at=datetime.utcnow()
            )
            results.append(result)

        return {
            "status": "success",
            "translations": results,
            "count": len(results),
            "timestamp": datetime.utcnow()
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error in batch translation: {str(e)}"
        )


@router.put("/update-cache/{content_id}")
async def update_translation_cache(content_id: str, content_type: str, new_content: str):
    """
    Update the translation cache when content changes.
    Ensures technical terms maintain accuracy when translated to Urdu as required.
    """
    try:
        # Update the translation cache
        updated_translation = await translation_service.update_translation_cache(
            content_id=content_id,
            content_type=content_type,
            new_content=new_content
        )

        return {
            "status": "success",
            "content_id": content_id,
            "content_type": content_type,
            "updated_translation": updated_translation,
            "timestamp": datetime.utcnow()
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating translation cache: {str(e)}"
        )


@router.get("/validate-accuracy/{content_id}")
async def validate_translation_accuracy(content_id: str, original_content: str, translated_content: str):
    """
    Validate that technical terms maintain accuracy when translated to Urdu.
    """
    try:
        # In a real implementation, this would check for technical accuracy
        # For this mock, we'll return a basic validation result

        # Check if certain technical terms are preserved appropriately
        technical_terms = ["ROS2", "Isaac", "Gazebo", "Python", "AI", "robotics", "algorithm"]
        preserved_count = 0

        for term in technical_terms:
            if term in original_content and term in translated_content:
                preserved_count += 1

        accuracy_score = preserved_count / len(technical_terms) if technical_terms else 0

        return {
            "content_id": content_id,
            "accuracy_score": accuracy_score,
            "technical_terms_preserved": preserved_count,
            "total_technical_terms": len(technical_terms),
            "validation_passed": accuracy_score >= 0.7,  # Threshold for accuracy
            "timestamp": datetime.utcnow()
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error validating translation accuracy: {str(e)}"
        )