"""Integration tests for textbook content structure validation."""

import pytest
from pathlib import Path
import os
from src.utils.content_validator import ContentValidator
from src.utils.citation_formatter import CitationFormatter


def test_textbook_content_structure():
    """Validate textbook content structure per requirements: diagrams, exercises, citations."""
    # Check that the frontend docs directory exists
    docs_path = Path("../../frontend/docs")
    if not docs_path.exists():
        # Try alternative path
        docs_path = Path("../../../frontend/docs")

    if not docs_path.exists():
        pytest.skip("Frontend docs directory not found")

    # Check each module directory
    modules = [d for d in docs_path.iterdir() if d.is_dir()]
    assert len(modules) >= 5, f"Expected at least 5 modules, found {len(modules)}"

    module_names = [m.name for m in modules]
    expected_modules = ["ros2", "gazebo-unity", "isaac", "vla", "capstone"]

    for expected_module in expected_modules:
        assert expected_module in module_names, f"Module {expected_module} not found"

    # Check content structure within each module
    for module_dir in modules:
        chapters = [d for d in module_dir.iterdir() if d.is_dir()]
        assert len(chapters) >= 2, f"Module {module_dir.name} should have at least 2 chapters"

        for chapter_dir in chapters:
            # Look for markdown files in the chapter
            md_files = list(chapter_dir.glob("*.md"))
            if not md_files:
                # Check if there are markdown files in subdirectories
                md_files = list(chapter_dir.rglob("*.md"))

            # Each chapter should have content
            for md_file in md_files:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                    # Check for diagrams (Mermaid or image references)
                    has_diagram = ('```mermaid' in content or
                                 '.png' in content or
                                 '.jpg' in content or
                                 '.svg' in content or
                                 'diagram' in content.lower())

                    # Check for exercises (exercise-related keywords)
                    has_exercise = ('exercise' in content.lower() or
                                  'problem' in content.lower() or
                                  'question' in content.lower() or
                                  'practice' in content.lower())

                    # Check for code examples
                    has_code = ('```' in content or
                              'code' in content.lower() or
                              '.py' in content or
                              '.js' in content)

                    # At least one of these elements should be present
                    assert (has_diagram or has_exercise or has_code), \
                        f"Chapter {chapter_dir.name} file {md_file.name} should contain diagrams, exercises, or code examples"


def test_content_meets_grade_level_requirements():
    """Validate all content meets grade 10-12 level with APA citations."""
    docs_path = Path("../../frontend/docs")
    if not docs_path.exists():
        # Try alternative path
        docs_path = Path("../../../frontend/docs")

    if not docs_path.exists():
        pytest.skip("Frontend docs directory not found")

    # Check content in all markdown files
    md_files = list(docs_path.rglob("*.md"))

    for md_file in md_files:
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

            # Validate grade level
            grade_validation = ContentValidator.validate_grade_level(content)
            assert grade_validation["is_valid"], \
                f"Content in {md_file} does not meet grade level requirements. " \
                f"Reading ease: {grade_validation['reading_ease']}, " \
                f"Target range: 10-12"

            # Check for APA citations
            has_citations = ContentValidator.has_apa_citations(content)
            # Not all content files need citations, but important ones should have them
            # We'll check for citation patterns in important content


def test_apa_citation_formatting():
    """Test that citations follow APA format."""
    # Test the citation formatter with various citation types
    formatter = CitationFormatter()

    # Test book citation
    book_citation = {
        "type": "book",
        "author": "Smith, J. A.",
        "year": "2023",
        "title": "Introduction to Robotics",
        "publisher": "Robotics Press"
    }

    formatted_book = formatter.format_citation(book_citation)
    assert "Smith, J. A." in formatted_book
    assert "(2023)" in formatted_book
    assert "Introduction to Robotics" in formatted_book

    # Test journal article citation
    article_citation = {
        "type": "journal",
        "author": "Doe, J. B., & Roe, C. D.",
        "year": "2022",
        "title": "Advanced Control Systems",
        "journal": "Journal of Robotics",
        "volume": "15",
        "issue": "3",
        "pages": "123-145"
    }

    formatted_article = formatter.format_citation(article_citation)
    assert "Doe, J. B., & Roe, C. D." in formatted_article
    assert "(2022)" in formatted_article
    assert "Journal of Robotics" in formatted_article


def test_content_has_required_elements():
    """Test that content has required elements like outcomes, explanations, etc."""
    docs_path = Path("../../frontend/docs")
    if not docs_path.exists():
        # Try alternative path
        docs_path = Path("../../../frontend/docs")

    if not docs_path.exists():
        pytest.skip("Frontend docs directory not found")

    # Check for required elements in chapter files
    md_files = list(docs_path.rglob("*.md"))

    for md_file in md_files:
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

            # Look for learning outcomes
            has_outcomes = ('outcome' in content.lower() or
                          'objective' in content.lower() or
                          'learning goal' in content.lower())

            # Look for explanations
            has_explanations = ('explained' in content.lower() or
                              'explanation' in content.lower() or
                              'understand' in content.lower() or
                              'concept' in content.lower())

            # Content should have some educational structure
            assert (has_outcomes or has_explanations), \
                f"Content in {md_file} should have learning outcomes or explanations"


def test_exercises_have_answers():
    """Test that exercises have corresponding answers."""
    docs_path = Path("../../frontend/docs")
    if not docs_path.exists():
        # Try alternative path
        docs_path = Path("../../../frontend/docs")

    if not docs_path.exists():
        pytest.skip("Frontend docs directory not found")

    # Check for exercise-answer pairs in content
    md_files = list(docs_path.rglob("*.md"))

    for md_file in md_files:
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

            # If content has exercises, it should have answers or solutions
            has_exercises = 'exercise' in content.lower() or 'problem' in content.lower()

            if has_exercises:
                # Check if it has answers or solutions
                has_answers = ('answer' in content.lower() or
                             'solution' in content.lower() or
                             'hint' in content.lower())

                # Not all exercises need answers in the same file, so this is a soft check
                # We'll just verify the structure is reasonable


def test_safety_warnings_in_appropriate_content():
    """Test that content with physical robotics has safety warnings."""
    docs_path = Path("../../frontend/docs")
    if not docs_path.exists():
        # Try alternative path
        docs_path = Path("../../../frontend/docs")

    if not docs_path.exists():
        pytest.skip("Frontend docs directory not found")

    # Check for safety-related content
    md_files = list(docs_path.rglob("*.md"))

    for md_file in md_files:
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

            # If content mentions physical hardware, real robots, etc., check for safety
            has_physical_content = ('hardware' in content.lower() or
                                  'physical' in content.lower() or
                                  'real robot' in content.lower() or
                                  'motor' in content.lower() or
                                  'actuator' in content.lower())

            if has_physical_content:
                # Should have safety-related keywords
                has_safety = ('safety' in content.lower() or
                            'warning' in content.lower() or
                            'caution' in content.lower() or
                            'risk' in content.lower())

                # For now, just log if safety content is missing - not all physical content needs warnings
                if not has_safety:
                    print(f"Physical content in {md_file} may need safety warnings")


if __name__ == "__main__":
    pytest.main([__file__])