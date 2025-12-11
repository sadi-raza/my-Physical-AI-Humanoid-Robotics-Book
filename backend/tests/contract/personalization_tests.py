"""API contract tests for personalization endpoints."""

import pytest
import asyncio
from fastapi.testclient import TestClient
from typing import Dict, Any
from src.api.main import app
from src.models.personalization import PersonalizationProfileCreate
from src.services.personalization_service import PersonalizationService


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


@pytest.mark.asyncio
async def test_store_user_background_contract():
    """Test the contract for storing user background."""
    client = TestClient(app)

    # Test request data structure
    personalization_request = {
        "user_id": "test_user_123",
        "background": "PhD in Robotics",
        "research_interests": ["computer vision", "manipulation"],
        "learning_goals": "Learn advanced humanoid control",
        "experience_level": "advanced",
        "preferred_modules": ["ros2", "isaac"]
    }

    response = client.post("/personalize/", json=personalization_request)

    # Contract assertions
    assert response.status_code == 200  # Assuming successful creation

    # Check response structure
    response_data = response.json()
    assert "id" in response_data
    assert "user_id" in response_data
    assert "personalization_settings" in response_data
    assert "learning_progress" in response_data
    assert "preference_tags" in response_data
    assert "created_at" in response_data
    assert "updated_at" in response_data

    # Check specific field values
    assert response_data["user_id"] == "test_user_123"
    assert "PhD in Robotics" in str(response_data["personalization_settings"]["background"])
    assert "computer vision" in response_data["personalization_settings"]["research_interests"]
    assert "advanced" == response_data["personalization_settings"]["experience_level"]


@pytest.mark.asyncio
async def test_get_user_personalization_profiles_contract():
    """Test the contract for retrieving user personalization profiles."""
    client = TestClient(app)
    user_id = "test_user_123"

    response = client.get(f"/personalize/{user_id}")

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert isinstance(response_data, list)

    if response_data:  # If profiles exist
        profile = response_data[0]
        assert "id" in profile
        assert "user_id" in profile
        assert "personalization_settings" in profile
        assert "learning_progress" in profile
        assert "preference_tags" in profile
        assert "created_at" in profile
        assert "updated_at" in profile


@pytest.mark.asyncio
async def test_update_personalization_profile_contract():
    """Test the contract for updating personalization profiles."""
    client = TestClient(app)
    profile_id = "test_profile_123"

    update_data = {
        "personalization_settings": {
            "experience_level": "intermediate"
        },
        "learning_progress": {
            "ros2_completed": True
        },
        "preference_tags": ["ROS2", "simulation"]
    }

    response = client.put(f"/personalize/{profile_id}", json=update_data)

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "id" in response_data
    assert "user_id" in response_data
    assert "personalization_settings" in response_data
    assert "learning_progress" in response_data
    assert "preference_tags" in response_data
    assert "created_at" in response_data
    assert "updated_at" in response_data


@pytest.mark.asyncio
async def test_personalize_content_contract():
    """Test the contract for content personalization."""
    client = TestClient(app)

    content_request = {
        "user_id": "test_user_123",
        "module_id": "ros2",
        "chapter_id": "ros2_basics",
        "content_type": "text",
        "context": {
            "content": "This is sample textbook content to be personalized."
        }
    }

    response = client.post("/personalize/personalize-content", json=content_request)

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "original_content" in response_data
    assert "personalized_content" in response_data
    assert "modification_reason" in response_data
    assert "confidence" in response_data

    # Check data types
    assert isinstance(response_data["original_content"], str)
    assert isinstance(response_data["personalized_content"], str)
    assert isinstance(response_data["modification_reason"], str)
    assert isinstance(response_data["confidence"], float)

    # Check confidence range
    assert 0.0 <= response_data["confidence"] <= 1.0


@pytest.mark.asyncio
async def test_get_recommended_modules_contract():
    """Test the contract for getting recommended modules."""
    client = TestClient(app)
    user_id = "test_user_123"

    response = client.get(f"/personalize/recommended-modules/{user_id}")

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "user_id" in response_data
    assert "recommended_order" in response_data
    assert "reasoning" in response_data

    # Check data types
    assert isinstance(response_data["user_id"], str)
    assert isinstance(response_data["recommended_order"], list)
    assert isinstance(response_data["reasoning"], str)


@pytest.mark.asyncio
async def test_track_learning_progress_contract():
    """Test the contract for tracking learning progress."""
    client = TestClient(app)

    progress_data = {
        "chapter_completed": True,
        "time_spent_seconds": 1200,
        "exercises_completed": 3,
        "exercises_total": 5,
        "completion_percentage": 60.0
    }

    response = client.post(
        "/personalize/track-progress?user_id=test_user_123&module_id=ros2&chapter_id=basics",
        json=progress_data
    )

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "status" in response_data
    assert "user_id" in response_data
    assert "module_id" in response_data
    assert "chapter_id" in response_data
    assert "progress_tracked" in response_data
    assert "updated_at" in response_data


@pytest.mark.asyncio
async def test_get_user_progress_contract():
    """Test the contract for getting user progress."""
    client = TestClient(app)
    user_id = "test_user_123"

    response = client.get(f"/personalize/progress/{user_id}")

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "user_id" in response_data
    assert "total_modules" in response_data
    assert "completed_modules" in response_data
    assert "in_progress_modules" in response_data
    assert "modules_progress" in response_data
    assert "overall_completion" in response_data
    assert "last_updated" in response_data

    # Check data types
    assert isinstance(response_data["total_modules"], int)
    assert isinstance(response_data["completed_modules"], int)
    assert isinstance(response_data["in_progress_modules"], int)
    assert isinstance(response_data["modules_progress"], dict)
    assert isinstance(response_data["overall_completion"], float)


def test_personalization_service_interface():
    """Test the personalization service interface contract."""
    service = PersonalizationService()

    # Check that required methods exist
    assert hasattr(service, 'personalize_content')
    assert hasattr(service, 'get_personalized_chapter')
    assert hasattr(service, 'cache_personalized_content')
    assert hasattr(service, 'get_cached_personalized_content')

    # Check that the service can be initialized
    assert service is not None


if __name__ == "__main__":
    pytest.main([__file__])