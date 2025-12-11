"""Skill for modifying content presentation based on user profile."""

from typing import Dict, Any
from ..services.personalization_service import PersonalizationService
from ..models.personalization import PersonalizationProfile


async def personalize_content_skill(
    content: str,
    profile: PersonalizationProfile,
    content_type: str = "text",
    module_id: str = "",
    chapter_id: str = ""
) -> Dict[str, Any]:
    """
    Skill to modify content presentation based on user profile.

    Args:
        content: Original content to personalize
        profile: User's personalization profile
        content_type: Type of content ('text', 'code', 'diagram', 'exercise', etc.)
        module_id: Module ID for context
        chapter_id: Chapter ID for context

    Returns:
        Dictionary with personalized content and modification details
    """
    personalization_service = PersonalizationService()

    # Personalize the content
    personalized_content = await personalization_service.personalize_content(
        content=content,
        profile=profile,
        content_type=content_type,
        module_id=module_id,
        chapter_id=chapter_id
    )

    return {
        "original_content": content,
        "personalized_content": personalized_content,
        "modification_reason": "Content personalized based on user profile",
        "confidence": 0.85
    }


def execute(
    content: str,
    profile: PersonalizationProfile,
    content_type: str = "text",
    module_id: str = "",
    chapter_id: str = ""
) -> Dict[str, Any]:
    """
    Execute the personalize-content skill synchronously.

    Args:
        content: Original content to personalize
        profile: User's personalization profile
        content_type: Type of content ('text', 'code', 'diagram', 'exercise', etc.)
        module_id: Module ID for context
        chapter_id: Chapter ID for context

    Returns:
        Dictionary with personalized content and modification details
    """
    import asyncio
    return asyncio.run(
        personalize_content_skill(content, profile, content_type, module_id, chapter_id)
    )