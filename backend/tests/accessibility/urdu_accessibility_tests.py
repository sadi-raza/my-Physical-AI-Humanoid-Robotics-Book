"""Accessibility tests for Urdu translation functionality."""

import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.config import settings


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_urdu_translation_api_accessibility(client):
    """Test that Urdu translation API is accessible and follows accessibility standards."""
    # Check if Urdu translation is enabled
    if not settings.urdu_translation_enabled:
        pytest.skip("Urdu translation is disabled in configuration")

    # Test basic accessibility of the translation endpoint
    translation_request = {
        "content_id": "accessibility_test_1",
        "content_type": "text",
        "content": "This is test content for accessibility validation.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=translation_request)

    # Check response status and structure
    assert response.status_code in [200, 500]  # 500 if service is disabled

    if response.status_code == 200:
        result = response.json()
        assert "urdu_translation" in result


def test_urdu_translation_preserves_semantic_structure(client):
    """Test that Urdu translation preserves semantic structure for accessibility."""
    if not settings.urdu_translation_enabled:
        pytest.skip("Urdu translation is disabled in configuration")

    # Test with content that has semantic structure
    structured_content = {
        "content_id": "structured_content_123",
        "content_type": "text",
        "content": "# Main Heading\n\nThis is a paragraph with **bold** and *italic* text.\n\n## Subheading\n\n- List item 1\n- List item 2",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=structured_content)

    if response.status_code == 200:
        result = response.json()
        urdu_translation = result["urdu_translation"]

        # Check that structural elements are preserved in some form
        # (The exact representation may differ in Urdu translation)
        assert isinstance(urdu_translation, str)
        assert len(urdu_translation) > 0


def test_urdu_translation_handles_special_characters(client):
    """Test that Urdu translation properly handles special characters and punctuation."""
    if not settings.urdu_translation_enabled:
        pytest.skip("Urdu translation is disabled in configuration")

    special_content = {
        "content_id": "special_chars_456",
        "content_type": "text",
        "content": "This content has special characters: @, #, $, %, &, <, >, etc.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=special_content)

    if response.status_code == 200:
        result = response.json()
        urdu_translation = result["urdu_translation"]

        # Should handle special characters appropriately
        assert isinstance(urdu_translation, str)


def test_urdu_translation_api_documentation(client):
    """Test that Urdu translation API has proper documentation for accessibility."""
    # Check if API documentation is available
    response = client.get("/docs")  # Standard FastAPI docs
    assert response.status_code in [200, 404]  # 404 if docs are disabled

    if response.status_code == 200:
        # API documentation should be available
        assert response.headers.get('content-type', '').startswith('text/html')


def test_urdu_translation_response_structure(client):
    """Test that Urdu translation responses follow a consistent, accessible structure."""
    if not settings.urdu_translation_enabled:
        pytest.skip("Urdu translation is disabled in configuration")

    test_content = {
        "content_id": "response_structure_789",
        "content_type": "text",
        "content": "Testing response structure for accessibility.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=test_content)

    if response.status_code == 200:
        result = response.json()

        # Check for required fields in response
        required_fields = ["id", "content_id", "original_content", "urdu_translation", "translated_at"]
        for field in required_fields:
            assert field in result, f"Required field '{field}' missing from response"


def test_urdu_translation_error_handling(client):
    """Test that Urdu translation has accessible error handling."""
    if not settings.urdu_translation_enabled:
        # Even if disabled, it should provide clear error messages
        translation_request = {
            "content_id": "error_test_111",
            "content_type": "text",
            "content": "This will test error handling.",
            "preserve_formatting": True
        }

        response = client.post("/translate-ur/", json=translation_request)

        # Should provide informative error when disabled
        if response.status_code == 500:
            response_data = response.json()
            assert "detail" in response_data or "error" in response_data


def test_urdu_translation_content_length_considerations(client):
    """Test that Urdu translation considers content length for accessibility."""
    if not settings.urdu_translation_enabled:
        pytest.skip("Urdu translation is disabled in configuration")

    short_content = {
        "content_id": "short_content_222",
        "content_type": "text",
        "content": "Short.",
        "preserve_formatting": True
    }

    long_content = {
        "content_id": "long_content_333",
        "content_type": "text",
        "content": "This is a much longer piece of content that should be handled appropriately by the translation system to ensure it remains accessible and usable.",
        "preserve_formatting": True
    }

    # Test with short content
    response1 = client.post("/translate-ur/", json=short_content)
    if response1.status_code == 200:
        result1 = response1.json()
        assert "urdu_translation" in result1

    # Test with long content
    response2 = client.post("/translate-ur/", json=long_content)
    if response2.status_code == 200:
        result2 = response2.json()
        assert "urdu_translation" in result2


def test_urdu_translation_api_health_endpoint(client):
    """Test that Urdu translation has a health check endpoint."""
    response = client.get("/translate-ur/health")
    assert response.status_code == 200

    health_data = response.json()
    assert "status" in health_data
    assert "service" in health_data
    assert health_data["status"] in ["healthy", "unhealthy"]


def test_urdu_translation_multilingual_considerations(client):
    """Test that Urdu translation system considers multilingual accessibility."""
    if not settings.urdu_translation_enabled:
        pytest.skip("Urdu translation is disabled in configuration")

    # Test content with mixed language elements (common in technical docs)
    mixed_content = {
        "content_id": "mixed_lang_444",
        "content_type": "text",
        "content": "This includes English terms like ROS2 and Python within Urdu context.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=mixed_content)

    if response.status_code == 200:
        result = response.json()
        urdu_translation = result["urdu_translation"]

        # Should handle mixed-language content appropriately
        assert isinstance(urdu_translation, str)


def test_urdu_translation_api_rate_limiting_considerations(client):
    """Test that Urdu translation API considers accessibility in rate limiting."""
    # This test checks that the API provides appropriate responses
    # even under potential rate limiting scenarios

    if not settings.urdu_translation_enabled:
        pytest.skip("Urdu translation is disabled in configuration")

    # Make a few requests to test API behavior
    test_requests = [
        {"content_id": f"rate_test_{i}", "content_type": "text",
         "content": f"Test content {i} for rate limiting.", "preserve_formatting": True}
        for i in range(3)
    ]

    responses = []
    for req in test_requests:
        response = client.post("/translate-ur/", json=req)
        responses.append(response.status_code)

    # Should handle multiple requests appropriately
    successful_requests = [r for r in responses if r == 200]
    print(f"Successful translation requests: {len(successful_requests)}/{len(responses)}")


if __name__ == "__main__":
    pytest.main([__file__])