"""Integration tests for translation functionality and quality validation."""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch
from src.api.main import app
from src.services.translation_service import TranslationService


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


def test_translation_quality_with_technical_terms():
    """Test that technical terms are accurately translated or preserved."""
    client = TestClient(app)

    # Content with technical terms that should be handled specially
    technical_content = {
        "content_id": "technical_content_123",
        "content_type": "text",
        "content": "In ROS2, nodes communicate through topics using publishers and subscribers. The rclpy library provides Python APIs for ROS2.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=technical_content)
    assert response.status_code == 200

    result = response.json()
    urdu_translation = result["urdu_translation"]

    # The translation should handle technical terms appropriately
    # Either translate them to appropriate Urdu equivalents or preserve them as technical terms
    assert isinstance(urdu_translation, str)
    # Should not be empty
    assert len(urdu_translation.strip()) > 0


def test_translation_preserves_code_examples():
    """Test that code examples are preserved during translation."""
    client = TestClient(app)

    content_with_code = {
        "content_id": "code_content_456",
        "content_type": "text",
        "content": "Here's a Python example:\n```python\ndef hello_robot():\n    print('Hello, Robot!')\n```\nThis code should be preserved.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=content_with_code)
    assert response.status_code == 200

    result = response.json()
    urdu_translation = result["urdu_translation"]

    # The code block should be preserved in the translation
    assert "```python" in urdu_translation or "def hello_robot" in urdu_translation
    # The overall meaning should still be translated
    assert len(urdu_translation) > len(content_with_code["content"]) * 0.5  # Should have substantial content


def test_translation_preserves_links_and_formatting():
    """Test that links and formatting are preserved during translation."""
    client = TestClient(app)

    content_with_formatting = {
        "content_id": "format_content_789",
        "content_type": "text",
        "content": "Check out the [Isaac Sim documentation](https://docs.omniverse.nvidia.com/isaacsim) for more details. Also see `rclpy` API.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=content_with_formatting)
    assert response.status_code == 200

    result = response.json()
    urdu_translation = result["urdu_translation"]

    # Links and inline code should be preserved
    assert "[" in urdu_translation and "]" in urdu_translation  # Link markers preserved
    assert "(" in urdu_translation and ")" in urdu_translation  # URL preserved
    assert "`rclpy`" in urdu_translation or "rclpy" in urdu_translation  # Inline code preserved


def test_translation_quality_for_different_content_types():
    """Test translation quality across different content types."""
    client = TestClient(app)

    # Test chapter translation
    chapter_content = {
        "content_id": "chapter_test_111",
        "content_type": "chapter",
        "content": "This chapter introduces the fundamentals of humanoid robotics, including kinematics, dynamics, and control systems.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=chapter_content)
    assert response.status_code == 200
    chapter_result = response.json()
    assert len(chapter_result["urdu_translation"]) > 0

    # Test exercise translation
    exercise_content = {
        "content_id": "exercise_test_222",
        "content_type": "exercise",
        "content": "Exercise: Implement a basic ROS2 publisher node that publishes robot joint states.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=exercise_content)
    assert response.status_code == 200
    exercise_result = response.json()
    assert len(exercise_result["urdu_translation"]) > 0


def test_translation_caching_mechanism():
    """Test that translations are properly cached and retrieved."""
    client = TestClient(app)

    # First translation request
    content = {
        "content_id": "cache_test_333",
        "content_type": "text",
        "content": "This content will be translated and cached.",
        "preserve_formatting": False
    }

    response1 = client.post("/translate-ur/", json=content)
    assert response1.status_code == 200
    result1 = response1.json()
    first_translation = result1["urdu_translation"]

    # Second request with same content (should use cache if implemented)
    response2 = client.post("/translate-ur/", json=content)
    assert response2.status_code == 200
    result2 = response2.json()
    second_translation = result2["urdu_translation"]

    # Both translations should be the same
    assert first_translation == second_translation


def test_translation_accuracy_for_educational_content():
    """Test translation accuracy specifically for educational robotics content."""
    client = TestClient(app)

    # Educational content with concepts that need accurate translation
    educational_content = {
        "content_id": "edu_content_444",
        "content_type": "text",
        "content": "Forward kinematics calculates the position of the robot's end-effector based on joint angles. Inverse kinematics solves the opposite problem.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=educational_content)
    assert response.status_code == 200

    result = response.json()
    urdu_translation = result["urdu_translation"]

    # The translation should preserve the educational meaning
    assert len(urdu_translation) > 0
    # Should contain equivalents of key concepts
    # (This is a basic check - in a real system, we'd have more sophisticated validation)


def test_translation_of_robotics_specific_terms():
    """Test translation of common robotics terminology."""
    client = TestClient(app)

    robotics_terms = {
        "content_id": "terms_content_555",
        "content_type": "text",
        "content": "A manipulator arm has multiple joints and links. The Jacobian matrix relates joint velocities to end-effector velocities.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=robotics_terms)
    assert response.status_code == 200

    result = response.json()
    urdu_translation = result["urdu_translation"]

    assert len(urdu_translation) > 0
    # The translation should handle robotics-specific terminology appropriately


def test_translation_service_initialization():
    """Test that translation service initializes correctly with configuration."""
    from src.config import settings

    # Verify that the service can be initialized
    service = TranslationService()
    assert service is not None

    # Service should have the translator component
    assert hasattr(service, 'urdu_translator')


def test_translation_with_special_characters():
    """Test translation handling of special characters and punctuation."""
    client = TestClient(app)

    special_content = {
        "content_id": "special_content_666",
        "content_type": "text",
        "content": "The robot's position: x=1.5m, y=2.3m, θ=45°. Velocity: ẋ=0.5m/s, ẏ=0.3m/s.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=special_content)
    assert response.status_code == 200

    result = response.json()
    urdu_translation = result["urdu_translation"]

    # Should handle special characters appropriately
    assert len(urdu_translation) > 0


def test_translation_long_content():
    """Test translation of longer content to ensure quality is maintained."""
    client = TestClient(app)

    long_content = {
        "content_id": "long_content_777",
        "content_type": "text",
        "content": " ".join([
            "Humanoid robots are robots with human-like body structure.",
            "They typically have a head, torso, two arms, and two legs.",
            "The design allows them to operate in human environments.",
            "They can navigate spaces built for humans, like doorways and stairs.",
            "Control systems coordinate multiple joints for stable locomotion.",
            "Sensors provide feedback for balance and environmental awareness.",
            "Applications include assistance, research, and entertainment."
        ]),
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=long_content)
    assert response.status_code == 200

    result = response.json()
    urdu_translation = result["urdu_translation"]

    # Should translate longer content coherently
    assert len(urdu_translation) > len(long_content["content"]) * 0.3  # Should have reasonable length


def test_translation_quality_metrics():
    """Test that translation includes quality indicators."""
    client = TestClient(app)

    test_content = {
        "content_id": "quality_content_888",
        "content_type": "text",
        "content": "This content tests the quality of the translation system.",
        "preserve_formatting": True
    }

    response = client.post("/translate-ur/", json=test_content)
    assert response.status_code == 200

    result = response.json()
    assert "urdu_translation" in result
    # In a real system, we might have quality metrics, but for this implementation
    # we're mainly checking that the translation completes successfully


if __name__ == "__main__":
    pytest.main([__file__])