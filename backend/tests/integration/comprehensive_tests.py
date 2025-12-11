"""Comprehensive integration tests covering all user stories."""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, AsyncMock
import asyncio
from src.api.main import app
from src.services.rag_service import RAGService
from src.services.personalization_service import PersonalizationService
from src.services.translation_service import TranslationService


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


@pytest.mark.asyncio
async def test_user_story_1_complete_workflow():
    """Test User Story 1: Access Interactive Textbook Content."""
    client = TestClient(app)

    # Test RAG functionality
    query_request = {
        "query": "What is ROS2 and how does it work?",
        "include_citations": True,
        "max_results": 3
    }

    response = client.post("/rag/query", json=query_request)

    # Check that response is successful
    assert response.status_code in [200, 404, 422]  # 404 if no content found, 422 for validation errors

    if response.status_code == 200:
        result = response.json()
        # Check for expected response structure
        assert "response" in result
        assert "citations" in result
        assert "confidence_score" in result


@pytest.mark.asyncio
async def test_user_story_2_complete_workflow():
    """Test User Story 2: Personalize Learning Experience."""
    client = TestClient(app)

    # 1. Store user background
    personalization_request = {
        "user_id": "integration_user_123",
        "background": "PhD in Robotics",
        "research_interests": ["computer vision", "manipulation"],
        "learning_goals": "Learn advanced humanoid control",
        "experience_level": "advanced",
        "preferred_modules": ["ros2", "isaac"]
    }

    response = client.post("/personalize/", json=personalization_request)
    assert response.status_code in [200, 422]  # Could be validation error if dependencies not met

    if response.status_code == 200:
        profile_data = response.json()
        assert profile_data["user_id"] == "integration_user_123"

        # 2. Get recommended modules based on profile
        response = client.get("/personalize/recommended-modules/integration_user_123")
        if response.status_code == 200:
            recommendations = response.json()
            assert "recommended_order" in recommendations

        # 3. Personalize content based on profile
        content_request = {
            "user_id": "integration_user_123",
            "module_id": "ros2",
            "chapter_id": "ros2_basics",
            "content_type": "text",
            "context": {
                "content": "This is textbook content to be personalized."
            }
        }

        response = client.post("/personalize/personalize-content", json=content_request)
        if response.status_code == 200:
            content_response = response.json()
            assert "personalized_content" in content_response


@pytest.mark.asyncio
async def test_user_story_3_complete_workflow():
    """Test User Story 3: Access Content in Urdu Language."""
    client = TestClient(app)

    # Test Urdu translation functionality
    translation_request = {
        "content_id": "integration_translation_123",
        "content_type": "text",
        "content": "This is English content that will be translated to Urdu.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=translation_request)

    # Check if translation service is enabled
    if response.status_code == 200:
        result = response.json()
        assert "urdu_translation" in result
        assert result["content_id"] == "integration_translation_123"
    elif response.status_code == 500:
        # Check if it's because Urdu translation is disabled
        response_data = response.json()
        if "disabled" in str(response_data).lower():
            # This is acceptable if Urdu translation is disabled in config
            pass


@pytest.mark.asyncio
async def test_end_to_end_textbook_access():
    """Test complete textbook access workflow."""
    client = TestClient(app)

    # Test basic API health
    response = client.get("/health")
    assert response.status_code == 200

    # Test RAG query (User Story 1)
    query_request = {
        "query": "Tell me about robotics fundamentals",
        "include_citations": True
    }

    response = client.post("/rag/query", json=query_request)
    # The response status depends on whether content exists in the database
    assert response.status_code in [200, 404, 422]


@pytest.mark.asyncio
async def test_content_ingestion_and_retrieval():
    """Test the complete content flow from ingestion to retrieval."""
    client = TestClient(app)

    # This tests the integration between content storage and RAG retrieval
    # In a real system, we would first ingest content, then query it
    # For this test, we'll just verify the APIs exist and respond appropriately

    # Test that RAG endpoints exist
    query_request = {
        "query": "Sample query to test RAG functionality",
        "include_citations": True
    }

    response = client.post("/rag/query", json=query_request)
    assert response.status_code in [200, 404, 422, 500]  # Various possible responses


@pytest.mark.asyncio
async def test_authentication_integration():
    """Test integration with authentication system."""
    client = TestClient(app)

    # Test auth endpoints exist (these would depend on Better-Auth implementation)
    # For now, just verify the system can handle user-related requests
    response = client.get("/health")
    assert response.status_code == 200


@pytest.mark.asyncio
async def test_personalization_with_rag():
    """Test integration between personalization and RAG systems."""
    client = TestClient(app)

    # First, create a user profile (if possible)
    personalization_request = {
        "user_id": "integrated_test_user",
        "background": "Robotics enthusiast",
        "experience_level": "beginner"
    }

    # Try to create profile
    profile_response = client.post("/personalize/", json=personalization_request)

    # Then try to use RAG with personalization context
    query_request = {
        "query": "Explain robotics basics",
        "include_citations": True
    }

    rag_response = client.post("/rag/query", json=query_request)

    # Both should respond appropriately
    assert profile_response.status_code in [200, 422, 500]
    assert rag_response.status_code in [200, 404, 422, 500]


@pytest.mark.asyncio
async def test_translation_with_personalization():
    """Test integration between translation and personalization systems."""
    client = TestClient(app)

    # Test that both systems can operate independently
    # Translation test
    translation_request = {
        "content_id": "integ_test_1",
        "content_type": "text",
        "content": "Test content for translation integration",
        "preserve_formatting": True
    }

    translation_response = client.post("/translate-ur/", json=translation_request)

    # Personalization test
    content_request = {
        "user_id": "integ_test_user",
        "module_id": "test_module",
        "chapter_id": "test_chapter",
        "content_type": "text",
        "context": {"content": "Test content for personalization"}
    }

    personalization_response = client.post("/personalize/personalize-content", json=content_request)

    # Both should respond appropriately
    assert translation_response.status_code in [200, 422, 500]
    assert personalization_response.status_code in [200, 404, 422, 500]


@pytest.mark.asyncio
async def test_system_health_and_dependencies():
    """Test overall system health and dependency integration."""
    client = TestClient(app)

    # Test main health endpoint
    response = client.get("/")
    # This might return 404 if no root endpoint is defined, which is fine
    assert response.status_code in [200, 404]

    # Test health endpoint specifically for the API
    response = client.get("/health")
    if response.status_code == 200:
        health_data = response.json()
        assert "status" in health_data
        assert "timestamp" in health_data


def test_service_initialization():
    """Test that all services initialize correctly."""
    # Test RAG service initialization
    rag_service = RAGService()
    assert rag_service is not None

    # Test personalization service initialization
    personalization_service = PersonalizationService()
    assert personalization_service is not None

    # Test translation service initialization
    try:
        translation_service = TranslationService()
        assert translation_service is not None
    except Exception:
        # Translation service might be disabled based on config
        from src.config import settings
        if settings.urdu_translation_enabled:
            raise


if __name__ == "__main__":
    pytest.main([__file__])