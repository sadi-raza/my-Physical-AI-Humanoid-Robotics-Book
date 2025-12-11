"""Personalization API endpoints for customizing textbook experience based on user background."""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import Optional, List, Dict, Any
from pydantic import BaseModel
from datetime import datetime
import uuid
from ..models.personalization import PersonalizationProfile, PersonalizationProfileCreate, PersonalizationProfileUpdate
from ..services.personalization_service import PersonalizationService

router = APIRouter(prefix="/personalize", tags=["personalization"])

# Mock database for demonstration
mock_personalization_db = {}


class PersonalizationRequest(BaseModel):
    user_id: str
    background: Optional[str] = None
    research_interests: Optional[List[str]] = []
    learning_goals: Optional[str] = None
    experience_level: Optional[str] = None  # beginner, intermediate, advanced, researcher
    preferred_modules: Optional[List[str]] = []  # List of module IDs


class PersonalizationResponse(BaseModel):
    user_id: str
    personalization_settings: Dict[str, Any]
    learning_progress: Dict[str, Any]
    preference_tags: List[str]
    updated_at: datetime


class ContentPersonalizationRequest(BaseModel):
    user_id: str
    module_id: str
    chapter_id: Optional[str] = None
    content_type: str  # 'text', 'code', 'diagram', 'exercise', etc.
    context: Optional[Dict[str, Any]] = {}


class PersonalizedContentResponse(BaseModel):
    original_content: str
    personalized_content: str
    modification_reason: str
    confidence: float


# Initialize personalization service
personalization_service = PersonalizationService()


@router.post("/", response_model=PersonalizationProfile)
async def store_user_background(personalization_request: PersonalizationRequest):
    """
    Store user background information for personalization.
    Implements FR-006: System MUST store user background information to enable chapter personalization.
    """
    try:
        # Create a personalization profile ID
        profile_id = str(uuid.uuid4())
        now = datetime.utcnow()

        # Create personalization profile
        profile = PersonalizationProfile(
            id=profile_id,
            user_id=personalization_request.user_id,
            module_id="all",  # This is a general profile for all modules
            personalization_settings={
                "background": personalization_request.background,
                "research_interests": personalization_request.research_interests,
                "learning_goals": personalization_request.learning_goals,
                "experience_level": personalization_request.experience_level,
                "preferred_modules": personalization_request.preferred_modules
            },
            learning_progress={},
            preference_tags=personalization_request.research_interests or [],
            created_at=now,
            updated_at=now
        )

        # Store in mock database
        mock_personalization_db[profile_id] = profile

        return profile

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error storing user background: {str(e)}"
        )


@router.get("/{user_id}", response_model=List[PersonalizationProfile])
async def get_user_personalization_profiles(user_id: str):
    """
    Retrieve all personalization profiles for a user.
    """
    try:
        user_profiles = [
            profile for profile in mock_personalization_db.values()
            if profile.user_id == user_id
        ]

        return user_profiles

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving personalization profiles: {str(e)}"
        )


@router.put("/{profile_id}", response_model=PersonalizationProfile)
async def update_personalization_profile(profile_id: str, update_data: PersonalizationProfileUpdate):
    """
    Update an existing personalization profile.
    """
    try:
        # Get existing profile
        if profile_id not in mock_personalization_db:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Personalization profile not found"
            )

        existing_profile = mock_personalization_db[profile_id]

        # Update the profile with provided data
        if update_data.personalization_settings is not None:
            existing_profile.personalization_settings.update(update_data.personalization_settings)

        if update_data.learning_progress is not None:
            existing_profile.learning_progress.update(update_data.learning_progress)

        if update_data.preference_tags is not None:
            existing_profile.preference_tags = update_data.preference_tags

        existing_profile.updated_at = datetime.utcnow()

        # Update in mock database
        mock_personalization_db[profile_id] = existing_profile

        return existing_profile

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating personalization profile: {str(e)}"
        )


@router.post("/personalize-content", response_model=PersonalizedContentResponse)
async def personalize_content(content_request: ContentPersonalizationRequest):
    """
    Modify content presentation based on user profile.
    Implements the personalize-content.skill requirement from the constitution.
    """
    try:
        # Get user's personalization profile
        user_profiles = [
            profile for profile in mock_personalization_db.values()
            if profile.user_id == content_request.user_id
        ]

        if not user_profiles:
            # If no profile exists, return original content
            return PersonalizedContentResponse(
                original_content=content_request.context.get("content", ""),
                personalized_content=content_request.context.get("content", ""),
                modification_reason="No personalization profile found for user",
                confidence=0.5
            )

        # Use the first profile (in a real system, might have multiple profiles per module)
        user_profile = user_profiles[0]

        # Use the personalization service to modify content
        personalized_content = await personalization_service.personalize_content(
            content=content_request.context.get("content", ""),
            profile=user_profile,
            content_type=content_request.content_type,
            module_id=content_request.module_id,
            chapter_id=content_request.chapter_id
        )

        return PersonalizedContentResponse(
            original_content=content_request.context.get("content", ""),
            personalized_content=personalized_content,
            modification_reason="Content personalized based on user profile",
            confidence=0.85  # High confidence in personalization
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error personalizing content: {str(e)}"
        )


@router.get("/recommended-modules/{user_id}")
async def get_recommended_modules(user_id: str):
    """
    Get recommended modules based on user's background and interests.
    """
    try:
        user_profiles = [
            profile for profile in mock_personalization_db.values()
            if profile.user_id == user_id
        ]

        if not user_profiles:
            # Return default module order if no profile exists
            return {
                "user_id": user_id,
                "recommended_order": ["ros2", "gazebo-unity", "isaac", "vla", "capstone"],
                "reasoning": "Default order for new users"
            }

        user_profile = user_profiles[0]
        experience_level = user_profile.personalization_settings.get("experience_level", "beginner")
        research_interests = user_profile.personalization_settings.get("research_interests", [])

        # Determine recommended order based on experience and interests
        recommended_order = []

        # Beginner users start with basics
        if experience_level in ["beginner", "intermediate"]:
            recommended_order = ["ros2", "gazebo-unity", "isaac", "vla", "capstone"]
        else:  # Advanced/researcher users might prefer different order
            if "simulation" in research_interests:
                recommended_order = ["gazebo-unity", "isaac", "ros2", "vla", "capstone"]
            elif "manipulation" in research_interests:
                recommended_order = ["ros2", "isaac", "vla", "gazebo-unity", "capstone"]
            else:
                recommended_order = ["ros2", "gazebo-unity", "isaac", "vla", "capstone"]

        return {
            "user_id": user_id,
            "recommended_order": recommended_order,
            "reasoning": f"Ordered based on experience level ({experience_level}) and interests ({research_interests})"
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting recommended modules: {str(e)}"
        )


@router.post("/track-progress")
async def track_learning_progress(user_id: str, module_id: str, chapter_id: str, progress_data: Dict[str, Any]):
    """
    Track user's learning progress for personalization.
    """
    try:
        # Find existing profile or create a new one
        user_profile = None
        for profile in mock_personalization_db.values():
            if profile.user_id == user_id and profile.module_id == module_id:
                user_profile = profile
                break

        if not user_profile:
            # Create a new profile for this module if it doesn't exist
            profile_id = str(uuid.uuid4())
            now = datetime.utcnow()

            user_profile = PersonalizationProfile(
                id=profile_id,
                user_id=user_id,
                module_id=module_id,
                personalization_settings={},
                learning_progress={},
                preference_tags=[],
                created_at=now,
                updated_at=now
            )
            mock_personalization_db[profile_id] = user_profile

        # Update learning progress
        progress_key = f"{module_id}:{chapter_id}"
        user_profile.learning_progress[progress_key] = {
            **progress_data,
            "updated_at": datetime.utcnow().isoformat()
        }
        user_profile.updated_at = datetime.utcnow()

        # Update in mock database
        for pid, profile in mock_personalization_db.items():
            if profile.id == user_profile.id:
                mock_personalization_db[pid] = user_profile
                break

        return {
            "status": "success",
            "user_id": user_id,
            "module_id": module_id,
            "chapter_id": chapter_id,
            "progress_tracked": progress_data,
            "updated_at": user_profile.updated_at
        }

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error tracking progress: {str(e)}"
        )


@router.get("/progress/{user_id}")
async def get_user_progress(user_id: str):
    """
    Get user's overall learning progress across all modules.
    """
    try:
        user_profiles = [
            profile for profile in mock_personalization_db.values()
            if profile.user_id == user_id
        ]

        progress_summary = {
            "user_id": user_id,
            "total_modules": 5,
            "completed_modules": 0,
            "in_progress_modules": 0,
            "modules_progress": {},
            "overall_completion": 0.0,
            "last_updated": datetime.utcnow()
        }

        for profile in user_profiles:
            # Count completed/in-progress modules based on learning progress
            module_completed = False
            module_in_progress = False

            for progress_key, progress_data in profile.learning_progress.items():
                if "completion_percentage" in progress_data:
                    if progress_data["completion_percentage"] >= 100:
                        module_completed = True
                    elif progress_data["completion_percentage"] > 0:
                        module_in_progress = True

            if module_completed:
                progress_summary["completed_modules"] += 1
            elif module_in_progress:
                progress_summary["in_progress_modules"] += 1

            progress_summary["modules_progress"][profile.module_id] = {
                "personalization_settings": profile.personalization_settings,
                "learning_progress": profile.learning_progress,
                "preference_tags": profile.preference_tags
            }

        progress_summary["overall_completion"] = (
            progress_summary["completed_modules"] / progress_summary["total_modules"]
        )

        return progress_summary

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting user progress: {str(e)}"
        )


@router.get("/health")
async def personalization_health_check():
    """
    Health check for the personalization service.
    """
    return {
        "status": "healthy",
        "service": "Personalization API",
        "timestamp": datetime.utcnow(),
        "features": {
            "user_background_storage": True,
            "content_personalization": True,
            "progress_tracking": True,
            "module_recommendation": True
        }
    }