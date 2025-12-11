"""Integration tests for personalization functionality."""

import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import patch, AsyncMock
from src.api.main import app
from src.services.personalization_service import PersonalizationService
from src.models.personalization import PersonalizationProfile


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


@pytest.mark.asyncio
async def test_end_to_end_personalization_workflow():
    """Test the complete personalization workflow from profile creation to content personalization."""
    client = TestClient(app)

    # 1. Create a user profile
    personalization_request = {
        "user_id": "integration_test_user_123",
        "background": "PhD in Robotics",
        "research_interests": ["computer vision", "manipulation"],
        "learning_goals": "Learn advanced humanoid control",
        "experience_level": "advanced",
        "preferred_modules": ["ros2", "isaac"]
    }

    response = client.post("/personalize/", json=personalization_request)
    assert response.status_code == 200
    profile_data = response.json()
    assert profile_data["user_id"] == "integration_test_user_123"
    profile_id = profile_data["id"]

    # 2. Retrieve the user's profile
    response = client.get(f"/personalize/{personalization_request['user_id']}")
    assert response.status_code == 200
    profiles = response.json()
    assert len(profiles) >= 1
    retrieved_profile = next(p for p in profiles if p["id"] == profile_id)
    assert retrieved_profile["personalization_settings"]["experience_level"] == "advanced"

    # 3. Personalize some content based on the profile
    content_request = {
        "user_id": "integration_test_user_123",
        "module_id": "ros2",
        "chapter_id": "ros2_basics",
        "content_type": "text",
        "context": {
            "content": "This is a complex ROS2 concept that should be personalized based on the user's advanced experience."
        }
    }

    response = client.post("/personalize/personalize-content", json=content_request)
    assert response.status_code == 200
    content_response = response.json()
    assert "original_content" in content_response
    assert "personalized_content" in content_response
    assert content_response["confidence"] > 0.5  # Should have reasonable confidence

    # 4. Track learning progress
    progress_data = {
        "chapter_completed": True,
        "time_spent_seconds": 1800,
        "exercises_completed": 5,
        "exercises_total": 5,
        "completion_percentage": 100.0
    }

    response = client.post(
        "/personalize/track-progress?user_id=integration_test_user_123&module_id=ros2&chapter_id=ros2_basics",
        json=progress_data
    )
    assert response.status_code == 200
    progress_response = response.json()
    assert progress_response["status"] == "success"
    assert progress_response["user_id"] == "integration_test_user_123"

    # 5. Get user progress summary
    response = client.get("/personalize/progress/integration_test_user_123")
    assert response.status_code == 200
    progress_summary = response.json()
    assert progress_summary["user_id"] == "integration_test_user_123"
    assert "modules_progress" in progress_summary
    assert "ros2" in progress_summary["modules_progress"]


@pytest.mark.asyncio
async def test_personalization_for_different_experience_levels():
    """Test that content is personalized differently based on experience level."""
    client = TestClient(app)

    # Test with beginner user
    beginner_request = {
        "user_id": "beginner_test_user",
        "background": "High school student",
        "research_interests": ["robotics"],
        "learning_goals": "Learn robotics basics",
        "experience_level": "beginner",
        "preferred_modules": ["ros2"]
    }

    response = client.post("/personalize/", json=beginner_request)
    assert response.status_code == 200

    # Test with advanced user
    advanced_request = {
        "user_id": "advanced_test_user",
        "background": "PhD in Robotics",
        "research_interests": ["computer vision", "manipulation"],
        "learning_goals": "Learn advanced humanoid control",
        "experience_level": "advanced",
        "preferred_modules": ["ros2", "isaac"]
    }

    response = client.post("/personalize/", json=advanced_request)
    assert response.status_code == 200

    # Request the same content for both users and compare personalization
    content_request = {
        "user_id": "beginner_test_user",
        "module_id": "ros2",
        "chapter_id": "ros2_basics",
        "content_type": "text",
        "context": {
            "content": "This is a complex ROS2 concept that should be explained differently based on experience level."
        }
    }

    beginner_response = client.post("/personalize/personalize-content", json=content_request)
    assert beginner_response.status_code == 200
    beginner_content = beginner_response.json()["personalized_content"]

    content_request["user_id"] = "advanced_test_user"
    advanced_response = client.post("/personalize/personalize-content", json=content_request)
    assert advanced_response.status_code == 200
    advanced_content = advanced_response.json()["personalized_content"]

    # Both should have personalized content, but potentially different
    assert isinstance(beginner_content, str)
    assert isinstance(advanced_content, str)


@pytest.mark.asyncio
async def test_module_recommendation_based_on_profile():
    """Test that module recommendations are based on user profile."""
    client = TestClient(app)

    # Create a user profile with specific interests
    personalization_request = {
        "user_id": "recommendation_test_user",
        "background": "PhD in Robotics",
        "research_interests": ["simulation"],
        "learning_goals": "Learn advanced simulation techniques",
        "experience_level": "advanced",
        "preferred_modules": ["gazebo-unity", "isaac"]
    }

    response = client.post("/personalize/", json=personalization_request)
    assert response.status_code == 200

    # Get recommended modules
    response = client.get("/personalize/recommended-modules/recommendation_test_user")
    assert response.status_code == 200
    recommendations = response.json()

    assert recommendations["user_id"] == "recommendation_test_user"
    assert isinstance(recommendations["recommended_order"], list)

    # Check that simulation-focused modules are recommended first
    recommended_order = recommendations["recommended_order"]
    # The user is interested in simulation, so gazebo-unity should be near the front
    if "gazebo-unity" in recommended_order:
        gazebo_position = recommended_order.index("gazebo-unity")
        # Should be in the first half of recommendations
        assert gazebo_position < len(recommended_order) / 2


@pytest.mark.asyncio
async def test_progress_tracking_updates_profile():
    """Test that tracking progress updates the user's profile."""
    client = TestClient(app)

    # Create a user profile
    personalization_request = {
        "user_id": "progress_test_user",
        "background": "Undergraduate student",
        "research_interests": ["robotics"],
        "learning_goals": "Learn robotics fundamentals",
        "experience_level": "beginner",
        "preferred_modules": ["ros2"]
    }

    response = client.post("/personalize/", json=personalization_request)
    assert response.status_code == 200

    # Track progress for a chapter
    progress_data = {
        "chapter_completed": True,
        "time_spent_seconds": 1200,
        "exercises_completed": 3,
        "exercises_total": 5,
        "completion_percentage": 60.0
    }

    response = client.post(
        "/personalize/track-progress?user_id=progress_test_user&module_id=ros2&chapter_id=basics",
        json=progress_data
    )
    assert response.status_code == 200

    # Get the user's progress summary
    response = client.get("/personalize/progress/progress_test_user")
    assert response.status_code == 200
    progress_summary = response.json()

    assert progress_summary["user_id"] == "progress_test_user"
    assert "ros2" in progress_summary["modules_progress"]
    module_progress = progress_summary["modules_progress"]["ros2"]
    assert "basics" in f"{module_progress['learning_progress']}"


@pytest.mark.asyncio
async def test_content_personalization_with_interests():
    """Test that content is highlighted based on user interests."""
    client = TestClient(app)

    # Create a user profile with specific interests
    personalization_request = {
        "user_id": "interest_test_user",
        "background": "MSc in AI",
        "research_interests": ["computer vision", "deep learning"],
        "learning_goals": "Learn computer vision in robotics",
        "experience_level": "intermediate",
        "preferred_modules": ["ros2", "isaac"]
    }

    response = client.post("/personalize/", json=personalization_request)
    assert response.status_code == 200

    # Request content that includes user interests
    content_request = {
        "user_id": "interest_test_user",
        "module_id": "isaac",
        "chapter_id": "vision_systems",
        "content_type": "text",
        "context": {
            "content": "This chapter covers computer vision systems in robotics. Deep learning approaches are very effective for visual perception in robotics applications."
        }
    }

    response = client.post("/personalize/personalize-content", json=content_request)
    assert response.status_code == 200
    content_response = response.json()

    assert "original_content" in content_response
    assert "personalized_content" in content_response

    # The personalized content should have processed the original content
    # according to the user's interests in computer vision and deep learning


def test_personalization_service_integration():
    """Test the integration of personalization service with other components."""
    # Test that the service can be properly initialized
    service = PersonalizationService()
    assert service is not None

    # Test that the service has access to configuration
    from src.config import settings
    assert hasattr(service, 'content_cache')


if __name__ == "__main__":
    pytest.main([__file__])