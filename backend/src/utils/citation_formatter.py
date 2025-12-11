"""Utility for formatting citations in APA style."""

from typing import Dict, Any, List
from datetime import datetime


class CitationFormatter:
    """Provides utilities for formatting citations in APA style as required by the project constitution."""

    @staticmethod
    def format_book_citation(title: str, author: str, year: int, publisher: str, location: str = None) -> str:
        """
        Format a book citation in APA style.

        Args:
            title: Book title
            author: Author name(s)
            year: Publication year
            publisher: Publisher name
            location: Location of publication (optional)

        Returns:
            Formatted citation string
        """
        if location:
            return f"{author} ({year}). {title}. {location}: {publisher}."
        else:
            return f"{author} ({year}). {title}. {publisher}."

    @staticmethod
    def format_journal_article(title: str, authors: List[str], year: int, journal: str,
                             volume: int, issue: int, pages: str, doi: str = None) -> str:
        """
        Format a journal article citation in APA style.

        Args:
            title: Article title
            authors: List of author names
            year: Publication year
            journal: Journal name
            volume: Volume number
            issue: Issue number
            pages: Page range
            doi: DOI identifier (optional)

        Returns:
            Formatted citation string
        """
        authors_str = ", ".join(authors)
        base_citation = f"{authors_str} ({year}). {title}. {journal}, {volume}({issue}), {pages}."

        if doi:
            base_citation += f" https://doi.org/{doi}"

        return base_citation

    @staticmethod
    def format_online_resource(title: str, author: str, year: int, url: str, access_date: str = None) -> str:
        """
        Format an online resource citation in APA style.

        Args:
            title: Resource title
            author: Author name
            year: Publication year
            url: URL of the resource
            access_date: Date the resource was accessed (optional)

        Returns:
            Formatted citation string
        """
        if access_date:
            return f"{author} ({year}). {title}. Retrieved from {url} on {access_date}"
        else:
            return f"{author} ({year}). {title}. Retrieved from {url}"

    @staticmethod
    def format_textbook_chapter(chapter_title: str, authors: List[str], year: int,
                               book_title: str, editors: List[str], publisher: str,
                               pages: str, location: str = None) -> str:
        """
        Format a textbook chapter citation in APA style.

        Args:
            chapter_title: Chapter title
            authors: List of chapter authors
            year: Publication year
            book_title: Book title
            editors: List of book editors
            publisher: Publisher name
            pages: Page range of the chapter
            location: Location of publication (optional)

        Returns:
            Formatted citation string
        """
        authors_str = ", ".join(authors)
        editors_str = ", ".join(editors)

        if location:
            return f"{authors_str} ({year}). {chapter_title}. In {editors_str} (Eds.), {book_title} (pp. {pages}). {location}: {publisher}."
        else:
            return f"{authors_str} ({year}). {chapter_title}. In {editors_str} (Eds.), {book_title} (pp. {pages}). {publisher}."

    @staticmethod
    def validate_peer_reviewed_source(source_metadata: Dict[str, Any]) -> bool:
        """
        Validate if a source meets the peer-reviewed requirement as specified in the constitution.

        Args:
            source_metadata: Dictionary containing source metadata

        Returns:
            True if the source is peer-reviewed, False otherwise
        """
        # Check if the source has the required peer-review indicators
        required_indicators = [
            'peer_reviewed',
            'refereed',
            'journal_type',
            'issn',
            'publisher_type'
        ]

        # Check if the source has peer_reviewed flag set to True
        if source_metadata.get('peer_reviewed', False):
            return True

        # Check if it's from a known peer-reviewed journal
        journal_type = source_metadata.get('journal_type', '').lower()
        if 'peer' in journal_type or 'refereed' in journal_type:
            return True

        # Check if it has an ISSN (indicating a formal journal)
        if source_metadata.get('issn'):
            return True

        # Check if it's from an academic publisher
        publisher_type = source_metadata.get('publisher_type', '').lower()
        if 'academic' in publisher_type or 'scholarly' in publisher_type or 'university' in publisher_type:
            return True

        return False

    @staticmethod
    def format_citation_from_metadata(metadata: Dict[str, Any]) -> str:
        """
        Format a citation based on the provided metadata.

        Args:
            metadata: Dictionary containing citation metadata

        Returns:
            Formatted citation string
        """
        # Determine the type of resource and format accordingly
        resource_type = metadata.get('type', 'generic')

        if resource_type == 'book':
            return CitationFormatter.format_book_citation(
                title=metadata.get('title', ''),
                author=metadata.get('author', ''),
                year=metadata.get('year', datetime.now().year),
                publisher=metadata.get('publisher', ''),
                location=metadata.get('location')
            )
        elif resource_type == 'journal_article':
            return CitationFormatter.format_journal_article(
                title=metadata.get('title', ''),
                authors=metadata.get('authors', []),
                year=metadata.get('year', datetime.now().year),
                journal=metadata.get('journal', ''),
                volume=metadata.get('volume', 1),
                issue=metadata.get('issue', 1),
                pages=metadata.get('pages', ''),
                doi=metadata.get('doi')
            )
        elif resource_type == 'online_resource':
            return CitationFormatter.format_online_resource(
                title=metadata.get('title', ''),
                author=metadata.get('author', ''),
                year=metadata.get('year', datetime.now().year),
                url=metadata.get('url', ''),
                access_date=metadata.get('access_date')
            )
        elif resource_type == 'textbook_chapter':
            return CitationFormatter.format_textbook_chapter(
                chapter_title=metadata.get('title', ''),
                authors=metadata.get('authors', []),
                year=metadata.get('year', datetime.now().year),
                book_title=metadata.get('book_title', ''),
                editors=metadata.get('editors', []),
                publisher=metadata.get('publisher', ''),
                pages=metadata.get('pages', ''),
                location=metadata.get('location')
            )
        else:
            # Generic format
            author = metadata.get('author', 'Unknown Author')
            title = metadata.get('title', 'Untitled')
            year = metadata.get('year', datetime.now().year)
            return f"{author} ({year}). {title}."