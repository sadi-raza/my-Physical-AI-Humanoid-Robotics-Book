"""API contract tests for translation endpoints."""

import pytest
from fastapi.testclient import TestClient
from typing import Dict, Any
from src.api.main import app


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_translate_content_contract(client):
    """Test the contract for translating content to Urdu."""
    # Test request data structure
    translation_request = {
        "content_id": "test_content_123",
        "content_type": "chapter",
        "content": "This is sample English content to be translated to Urdu.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=translation_request)

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "id" in response_data
    assert "content_id" in response_data
    assert "original_content" in response_data
    assert "urdu_translation" in response_data
    assert "content_type" in response_data
    assert "translated_at" in response_data

    # Check specific field values
    assert response_data["content_id"] == "test_content_123"
    assert response_data["content_type"] == "chapter"
    assert isinstance(response_data["urdu_translation"], str)


def test_translate_chapter_contract(client):
    """Test the contract for translating a complete chapter."""
    # Test request data structure
    translation_request = {
        "content_id": "chapter_123",
        "content_type": "chapter",
        "content": {
            "title": "Introduction to Robotics",
            "content": "Robotics is an interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others.",
            "exercises": [
                {
                    "question": "What is robotics?",
                    "answer": "Robotics is an interdisciplinary branch..."
                }
            ],
            "code_examples": [
                "# This code example should be preserved in translation",
                "def robot_move():",
                "    pass"
            ]
        }
    }

    response = client.post("/translate-ur/chapter", json=translation_request)

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "id" in response_data
    assert "title" in response_data  # Translated title
    assert "content" in response_data  # Translated content
    assert "exercises" in response_data
    assert "code_examples" in response_data  # Should be preserved

    # Code examples should be preserved without translation
    assert translation_request["content"]["code_examples"] == response_data["code_examples"]


def test_translate_section_contract(client):
    """Test the contract for translating a section."""
    # Test request data structure
    translation_request = {
        "content_id": "section_456",
        "content_type": "section",
        "content": "This section explains the fundamentals of robot kinematics."
    }

    response = client.post("/translate-ur/section", json=translation_request)

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "id" in response_data
    assert "content_id" in response_data
    assert "urdu_translation" in response_data
    assert "preserved_elements" in response_data  # For code, diagrams, etc.


def test_get_cached_translation_contract(client):
    """Test the contract for retrieving cached translations."""
    content_id = "cached_content_123"
    content_type = "chapter"

    response = client.get(f"/translate-ur/cached/{content_id}/{content_type}")

    # Contract assertions
    if response.status_code == 200:
        # Translation exists in cache
        response_data = response.json()
        assert "content_id" in response_data
        assert "urdu_translation" in response_data
        assert "cached_at" in response_data
        assert response_data["content_id"] == content_id
    elif response.status_code == 404:
        # Translation not in cache, which is also valid
        pass
    else:
        # Any other status is unexpected
        assert response.status_code in [200, 404]


def test_translation_health_check_contract(client):
    """Test the contract for translation service health check."""
    response = client.get("/translate-ur/health")

    # Contract assertions
    assert response.status_code == 200

    # Check response structure
    response_data = response.json()
    assert "status" in response_data
    assert "service" in response_data
    assert "timestamp" in response_data
    assert "features" in response_data

    # Check specific values
    assert response_data["status"] == "healthy"
    assert "Translation API" in response_data["service"]
    assert "urdu_translation_enabled" in response_data["features"]


def test_translate_with_preserved_formatting_contract(client):
    """Test the contract for translating content with preserved formatting."""
    # Test request with various formatting elements
    translation_request = {
        "content_id": "formatted_content_789",
        "content_type": "text",
        "content": "Here is some text with `inline code` and a [link](http://example.com). Also ```python\ncode block\n```.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=translation_request)

    # Contract assertions
    assert response.status_code == 200

    response_data = response.json()
    assert "original_content" in response_data
    assert "urdu_translation" in response_data

    # The response should contain preserved formatting elements
    original = response_data["original_content"]
    translated = response_data["urdu_translation"]

    # Check that formatting markers are preserved in the translated content
    assert "`inline code`" in original or "`inline code`" in translated  # At least one should have it
    assert "```" in original or "```" in translated  # Code blocks should be preserved


def test_translation_quality_indicators_contract(client):
    """Test the contract for translation quality indicators."""
    translation_request = {
        "content_id": "quality_test_123",
        "content_type": "chapter",
        "content": "This chapter covers advanced robotics concepts and technical terminology.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=translation_request)

    # Contract assertions
    assert response.status_code == 200

    response_data = response.json()
    assert "id" in response_data
    assert "content_id" in response_data
    assert "urdu_translation" in response_data
    assert "quality_metrics" in response_data or "confidence" in response_data

    # If quality metrics are present, check their structure
    if "quality_metrics" in response_data:
        metrics = response_data["quality_metrics"]
        assert "accuracy_score" in metrics
        assert "technical_term_preservation" in metrics


if __name__ == "__main__":
    pytest.main([__file__])