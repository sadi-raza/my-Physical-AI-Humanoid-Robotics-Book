"""Utility for validating content meets grade 10-12 level and quality requirements."""

import re
from typing import Dict, Any, List
from textstat import flesch_reading_ease, flesch_kincaid_grade


class ContentValidator:
    """Validates that content meets the grade 10-12 level educational standards as required by the project constitution."""

    @staticmethod
    def validate_grade_level(content: str) -> Dict[str, Any]:
        """
        Validates that content is appropriate for grade 10-12 level.

        Args:
            content: Text content to validate

        Returns:
            Dictionary with validation results
        """
        # Calculate readability scores
        reading_ease = flesch_reading_ease(content)
        grade_level = flesch_kincaid_grade(content)

        # Grade level ranges (Flesch-Kincaid Grade Level)
        # Grade 10-12 typically corresponds to ages 15-18
        # Grade level should be between 9-12 for grade 10-12 appropriateness
        is_appropriate = 9.0 <= grade_level <= 12.0

        # Reading ease score (Flesch Reading Ease)
        # 60-70 is considered "Plain English" which is appropriate for grade 10-12
        reading_ease_appropriate = 60 <= reading_ease <= 80

        return {
            "is_valid": is_appropriate,
            "grade_level": round(grade_level, 2),
            "reading_ease": round(reading_ease, 2),
            "reading_ease_appropriate": reading_ease_appropriate,
            "suggestions": ContentValidator._get_reading_level_suggestions(grade_level, reading_ease)
        }

    @staticmethod
    def _get_reading_level_suggestions(grade_level: float, reading_ease: float) -> List[str]:
        """Generate suggestions to improve reading level."""
        suggestions = []

        if grade_level > 12:
            suggestions.append("Consider simplifying language to lower the grade level")
            suggestions.append("Use shorter sentences and simpler vocabulary")
        elif grade_level < 9:
            suggestions.append("Consider increasing complexity for grade 10-12 level")

        if reading_ease < 60:
            suggestions.append("Content is difficult to read; consider simplifying")
        elif reading_ease > 80:
            suggestions.append("Content is very easy to read; consider adding more complexity")

        return suggestions

    @staticmethod
    def validate_exercises_presence(content: str, min_exercises: int = 2) -> Dict[str, Any]:
        """
        Validates that content includes at least the required number of exercises.

        Args:
            content: Text content to validate
            min_exercises: Minimum number of exercises required (default 2)

        Returns:
            Dictionary with validation results
        """
        # Look for exercise indicators in the content
        exercise_patterns = [
            r'exercise[s]?\s*\d+',  # Exercise 1, Exercises 1-5
            r'problem[s]?\s*\d+',  # Problem 1, Problems 1-5
            r'question[s]?\s*\d+',  # Question 1, Questions 1-5
            r'practice',  # Practice section
            r'quiz',  # Quiz section
            r'activity',  # Activity section
            r'assignment[s]?',  # Assignment section
        ]

        found_exercises = 0
        for pattern in exercise_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            found_exercises += len(matches)

        # Also look for exercise headers
        header_patterns = [
            r'#\s*Exercises?',  # Markdown header for exercises
            r'##\s*Exercises?',
            r'###\s*Exercises?',
        ]

        for pattern in header_patterns:
            matches = re.findall(pattern, content)
            found_exercises += len(matches) * 2  # Count each header as multiple exercises

        return {
            "has_required_exercises": found_exercises >= min_exercises,
            "found_exercises": found_exercises,
            "required_exercises": min_exercises
        }

    @staticmethod
    def validate_diagrams_presence(content: str, min_diagrams: int = 1) -> Dict[str, Any]:
        """
        Validates that content includes at least the required number of diagrams.

        Args:
            content: Text content to validate
            min_diagrams: Minimum number of diagrams required (default 1)

        Returns:
            Dictionary with validation results
        """
        # Look for diagram indicators in the content
        diagram_patterns = [
            r'diagram[s]?',  # Diagram references
            r'figure[s]?\s*\d+',  # Figure 1, Figures 1-5
            r'image[s]?\s*\d+',  # Image 1, Images 1-5
            r'chart[s]?',  # Chart references
            r'graph[s]?',  # Graph references
            r'plot[s]?',  # Plot references
            r'visualization[s]?',  # Visualization references
            r'flowchart[s]?',  # Flowchart references
            r'infographic[s]?',  # Infographic references
        ]

        found_diagrams = 0
        for pattern in diagram_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            found_diagrams += len(matches)

        # Look for markdown image syntax
        markdown_images = re.findall(r'!\[.*?\]\(.*?\)', content)
        found_diagrams += len(markdown_images)

        # Look for Mermaid diagrams
        mermaid_blocks = re.findall(r'```mermaid\s*.*?\s*```', content, re.DOTALL)
        found_diagrams += len(mermaid_blocks)

        return {
            "has_required_diagrams": found_diagrams >= min_diagrams,
            "found_diagrams": found_diagrams,
            "required_diagrams": min_diagrams
        }

    @staticmethod
    def validate_code_examples(content: str) -> Dict[str, Any]:
        """
        Validates that content includes code examples.

        Args:
            content: Text content to validate

        Returns:
            Dictionary with validation results
        """
        # Look for code indicators in the content
        code_patterns = [
            r'```.*?```',  # Markdown code blocks
            r'`.*?`',  # Inline code
            r'class\s+\w+',  # Class definitions
            r'def\s+\w+\s*\(',  # Function definitions
            r'function\s+\w+\s*\(',  # Function definitions
            r'import\s+\w+',  # Import statements
            r'#\s*.*',  # Comment lines
        ]

        found_code_elements = 0
        for pattern in code_patterns:
            if '```' in pattern:
                # Special handling for code blocks
                matches = re.findall(pattern, content, re.DOTALL)
            else:
                matches = re.findall(pattern, content)
            found_code_elements += len(matches)

        # Check for specific robotics/ROS/Isaac code patterns
        robotics_code_patterns = [
            r'rclpy',  # ROS2 Python client library
            r'ros2',  # ROS2 references
            r'gazebo',  # Gazebo simulator
            r'isaac',  # Isaac references
            r'node',  # ROS nodes
            r'topic',  # ROS topics
            r'service',  # ROS services
            r'action',  # ROS actions
            r'launch',  # ROS launch files
        ]

        found_robotics_code = 0
        for pattern in robotics_code_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            found_robotics_code += len(matches)

        return {
            "has_code_examples": found_code_elements > 0,
            "code_elements_count": found_code_elements,
            "robotics_code_references": found_robotics_code
        }

    @staticmethod
    def validate_citations(content: str) -> Dict[str, Any]:
        """
        Validates that content includes proper citations.

        Args:
            content: Text content to validate

        Returns:
            Dictionary with validation results
        """
        # Look for citation patterns (APA style)
        citation_patterns = [
            r'\(\w+\s*,\s*\d{4}\)',  # (Author, Year)
            r'\(\w+\s*&\s*\w+\s*,\s*\d{4}\)',  # (Author & Author, Year)
            r'\(\w+\s*et al\.,\s*\d{4}\)',  # (Author et al., Year)
            r'\[\d+\]',  # [1], [2], etc.
            r'\[.*?;\s*.*?\d{4}.*?\]',  # Multiple citations [Author 1; Author 2, Year]
        ]

        found_citations = 0
        for pattern in citation_patterns:
            matches = re.findall(pattern, content)
            found_citations += len(matches)

        # Look for reference list indicators
        reference_patterns = [
            r'References\s*',  # References header
            r'Bibliography\s*',  # Bibliography header
            r'Works Cited\s*',  # Works Cited header
        ]

        found_references = 0
        for pattern in reference_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            found_references += len(matches)

        return {
            "has_citations": found_citations > 0,
            "citation_count": found_citations,
            "has_reference_section": found_references > 0
        }

    @staticmethod
    def validate_safety_warnings(content: str) -> Dict[str, Any]:
        """
        Validates that content includes safety warnings for robotics content.

        Args:
            content: Text content to validate

        Returns:
            Dictionary with validation results
        """
        # Look for safety-related terms
        safety_patterns = [
            r'safety',  # Safety mentions
            r'warning[s]?',  # Warning mentions
            r'caution',  # Caution mentions
            r'risk[s]?',  # Risk mentions
            r'hazard[s]?',  # Hazard mentions
            r'danger',  # Danger mentions
            r'protective equipment',  # Protective equipment
            r'proper equipment',  # Proper equipment
            r'unsupported hardware',  # Unsupported hardware warnings
            r'physical risk[s]?',  # Physical risk mentions
            r'hardware requirement[s]?',  # Hardware requirements
        ]

        found_safety_mentions = 0
        for pattern in safety_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            found_safety_mentions += len(matches)

        return {
            "has_safety_content": found_safety_mentions > 0,
            "safety_mentions_count": found_safety_mentions
        }

    @staticmethod
    def validate_content_quality(content: str) -> Dict[str, Any]:
        """
        Validates that content meets all quality requirements from the constitution.

        Args:
            content: Text content to validate

        Returns:
            Dictionary with comprehensive validation results
        """
        grade_validation = ContentValidator.validate_grade_level(content)
        exercise_validation = ContentValidator.validate_exercises_presence(content)
        diagram_validation = ContentValidator.validate_diagrams_presence(content)
        code_validation = ContentValidator.validate_code_examples(content)
        citation_validation = ContentValidator.validate_citations(content)
        safety_validation = ContentValidator.validate_safety_warnings(content)

        all_validations = {
            "grade_level_valid": grade_validation["is_valid"],
            "exercises_valid": exercise_validation["has_required_exercises"],
            "diagrams_valid": diagram_validation["has_required_diagrams"],
            "code_examples_present": code_validation["has_code_examples"],
            "citations_present": citation_validation["has_citations"],
            "safety_content_present": safety_validation["has_safety_content"]
        }

        all_valid = all(all_validations.values())

        return {
            "is_valid": all_valid,
            "overall_score": sum(all_validations.values()) / len(all_validations),
            "grade_validation": grade_validation,
            "exercise_validation": exercise_validation,
            "diagram_validation": diagram_validation,
            "code_validation": code_validation,
            "citation_validation": citation_validation,
            "safety_validation": safety_validation,
            "validation_summary": all_validations
        }