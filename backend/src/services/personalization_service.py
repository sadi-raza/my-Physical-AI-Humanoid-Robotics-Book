"""Personalization service for modifying content presentation based on user profile."""

from typing import Dict, Any, List, Optional
from ..models.personalization import PersonalizationProfile
from ..config import settings
import re


class PersonalizationService:
    """Service for personalizing textbook content based on user background and preferences."""

    def __init__(self):
        # Check if personalization is enabled
        if not settings.personalization_enabled:
            raise Exception("Personalization is disabled in configuration")

        # In a real implementation, this might connect to a database or ML model
        self.content_cache = {}  # Simple cache for personalized content

    async def personalize_content(
        self,
        content: str,
        profile: PersonalizationProfile,
        content_type: str,
        module_id: str,
        chapter_id: Optional[str] = None
    ) -> str:
        """
        Personalize content based on user profile.

        Args:
            content: Original content to personalize
            profile: User's personalization profile
            content_type: Type of content ('text', 'code', 'diagram', 'exercise', etc.)
            module_id: Module ID for context
            chapter_id: Chapter ID for context

        Returns:
            Personalized content string
        """
        # Get user's experience level and interests from profile
        experience_level = profile.personalization_settings.get('experience_level', 'beginner')
        research_interests = profile.personalization_settings.get('research_interests', [])

        # Apply personalization based on content type
        if content_type == 'text':
            return await self._personalize_text_content(content, experience_level, research_interests)
        elif content_type == 'code':
            return await self._personalize_code_content(content, experience_level, research_interests)
        elif content_type == 'exercise':
            return await self._personalize_exercise_content(content, experience_level, research_interests)
        elif content_type == 'diagram':
            return await self._personalize_diagram_content(content, experience_level, research_interests)
        else:
            # For other types, apply general personalization
            return await self._apply_general_personalization(content, experience_level, research_interests)

    async def _personalize_text_content(
        self,
        content: str,
        experience_level: str,
        research_interests: List[str]
    ) -> str:
        """
        Personalize text content based on user's experience level and interests.
        """
        personalized_content = content

        # Adjust complexity based on experience level
        if experience_level == 'beginner':
            # Simplify complex concepts, add more explanations
            personalized_content = self._simplify_content(personalized_content)
        elif experience_level == 'advanced' or experience_level == 'researcher':
            # Add more technical depth and references
            personalized_content = self._add_technical_depth(personalized_content)

        # Highlight content related to user's research interests
        for interest in research_interests:
            # Make content related to interests more prominent
            pattern = r'(?i)\b' + re.escape(interest) + r'\b'
            personalized_content = re.sub(
                pattern,
                f"**{interest.upper()}**",  # Make interesting topics bold
                personalized_content
            )

        # Add context based on experience level
        if experience_level == 'beginner':
            # Add more context and background information
            personalized_content = self._add_beginner_context(personalized_content)
        elif experience_level in ['advanced', 'researcher']:
            # Add references to research papers and advanced concepts
            personalized_content = self._add_advanced_context(personalized_content)

        return personalized_content

    async def _personalize_code_content(
        self,
        content: str,
        experience_level: str,
        research_interests: List[str]
    ) -> str:
        """
        Personalize code content based on user's experience level and interests.
        """
        personalized_content = content

        # Add more comments for beginners
        if experience_level == 'beginner':
            personalized_content = self._add_code_comments(personalized_content)
        elif experience_level in ['advanced', 'researcher']:
            # Add more sophisticated examples
            personalized_content = self._add_advanced_code_examples(personalized_content)

        # Adjust complexity based on experience
        if experience_level == 'beginner':
            personalized_content = self._simplify_code(personalized_content)
        elif experience_level in ['advanced', 'researcher']:
            personalized_content = self._add_complexity(personalized_content)

        # Highlight code related to user's interests
        for interest in research_interests:
            if interest.lower() in content.lower():
                # Add special annotations for interesting code
                personalized_content += f"\n# RELEVANT TO YOUR INTEREST IN {interest.upper()}"

        return personalized_content

    async def _personalize_exercise_content(
        self,
        content: str,
        experience_level: str,
        research_interests: List[str]
    ) -> str:
        """
        Personalize exercises based on user's experience level and interests.
        """
        personalized_content = content

        if experience_level == 'beginner':
            # Make exercises more guided with hints
            personalized_content = self._add_exercise_hints(personalized_content)
        elif experience_level in ['advanced', 'researcher']:
            # Add more challenging extensions
            personalized_content = self._add_challenging_extensions(personalized_content)

        # Relate exercises to user's research interests
        for interest in research_interests:
            if interest.lower() in content.lower():
                personalized_content += f"\n*This exercise relates to your interest in {interest}*"

        return personalized_content

    async def _personalize_diagram_content(
        self,
        content: str,
        experience_level: str,
        research_interests: List[str]
    ) -> str:
        """
        Personalize diagram content based on user's experience level and interests.
        """
        # For diagram content, we might return different versions or annotations
        if experience_level == 'beginner':
            return content + " (Beginner-friendly version with simplified labels)"
        elif experience_level in ['advanced', 'researcher']:
            return content + " (Advanced version with technical annotations)"

        return content

    async def _apply_general_personalization(
        self,
        content: str,
        experience_level: str,
        research_interests: List[str]
    ) -> str:
        """
        Apply general personalization to any content type.
        """
        personalized_content = content

        # Add personalized introduction based on experience level
        if experience_level == 'beginner':
            intro = f"Hi! Since you're new to this topic, I've prepared this content with extra explanations:\n\n"
            personalized_content = intro + personalized_content
        elif experience_level in ['advanced', 'researcher']:
            intro = f"Given your advanced knowledge, here's a more technical perspective:\n\n"
            personalized_content = intro + personalized_content

        # Add relevant context based on interests
        for interest in research_interests:
            if interest.lower() in content.lower():
                context = f"\n\n*Note: This connects to your interest in {interest}*\n"
                personalized_content = personalized_content.replace("\n", context, 1)

        return personalized_content

    def _simplify_content(self, content: str) -> str:
        """Simplify content for beginners."""
        # Add more explanations and simpler terminology
        content = re.sub(r'(\w+) is a complex concept', r'\1 is a concept', content)
        content = re.sub(r'(\w+) can be understood as', r'\1 is', content)
        return content

    def _add_technical_depth(self, content: str) -> str:
        """Add more technical depth for advanced users."""
        # Add more technical details and references
        content += "\n\nFor more technical details, see advanced references."
        return content

    def _add_beginner_context(self, content: str) -> str:
        """Add more context for beginners."""
        # Add more background information
        content += "\n\n**Background**: This concept builds on previous knowledge."
        return content

    def _add_advanced_context(self, content: str) -> str:
        """Add advanced context for experienced users."""
        # Add references to research and advanced concepts
        content += "\n\n**Research Context**: See recent papers for implementation variations."
        return content

    def _add_code_comments(self, content: str) -> str:
        """Add more explanatory comments to code."""
        # This is a simplified example - in practice, would parse code and add appropriate comments
        return content.replace("# TODO:", "# Explanation: This is where we handle the main logic")

    def _add_advanced_code_examples(self, content: str) -> str:
        """Add advanced code examples."""
        return content + "\n# Advanced implementation with optimizations"

    def _simplify_code(self, content: str) -> str:
        """Simplify code for beginners."""
        return content.replace("# Optimized version", "# Simple version for learning")

    def _add_complexity(self, content: str) -> str:
        """Add complexity for advanced users."""
        return content.replace("# Basic implementation", "# Advanced implementation with error handling")

    def _add_exercise_hints(self, content: str) -> str:
        """Add hints to exercises for beginners."""
        return content + "\n\n**Hint**: Start by reviewing the previous examples."

    def _add_challenging_extensions(self, content: str) -> str:
        """Add challenging extensions to exercises for advanced users."""
        return content + "\n\n**Challenge Extension**: Implement this with additional constraints."

    async def get_personalized_chapter(self, chapter_id: str, profile: PersonalizationProfile) -> Dict[str, Any]:
        """
        Get a personalized version of a chapter based on user profile.

        Args:
            chapter_id: ID of the chapter to personalize
            profile: User's personalization profile

        Returns:
            Dictionary with personalized chapter content
        """
        # This would typically fetch the chapter from a database
        # For this implementation, we'll return a structure that would hold personalized content
        from ..models.chapter import Chapter

        # In a real implementation, we would fetch the chapter from the database
        # For now, we'll create a placeholder response
        personalized_chapter = {
            "id": chapter_id,
            "title": f"Personalized Chapter {chapter_id}",
            "content": f"Personalized content for chapter {chapter_id}",
            "profile_used": {
                "experience_level": profile.personalization_settings.get('experience_level', 'beginner'),
                "research_interests": profile.personalization_settings.get('research_interests', [])
            },
            "personalization_applied": True,
            "personalization_timestamp": datetime.utcnow().isoformat()
        }

        # Apply personalization to different parts of the chapter
        # This would involve calling personalize_content for each section
        return personalized_chapter

    async def cache_personalized_content(self, cache_key: str, content: str, profile_id: str):
        """
        Cache personalized content to avoid recomputation.

        Args:
            cache_key: Unique key for the content
            content: Personalized content to cache
            profile_id: ID of the profile used for personalization
        """
        cache_entry = {
            "content": content,
            "profile_id": profile_id,
            "timestamp": datetime.utcnow().isoformat(),
            "ttl": settings.cache_ttl_seconds  # Use configured TTL
        }
        self.content_cache[cache_key] = cache_entry

    async def get_cached_personalized_content(self, cache_key: str) -> Optional[str]:
        """
        Retrieve cached personalized content if still valid.

        Args:
            cache_key: Key for the cached content

        Returns:
            Cached content if valid, None otherwise
        """
        if cache_key in self.content_cache:
            cache_entry = self.content_cache[cache_key]
            # Check if cache is still valid (simplified check)
            # In a real implementation, we'd check the TTL
            return cache_entry["content"]
        return None


from datetime import datetime
from typing import Optional