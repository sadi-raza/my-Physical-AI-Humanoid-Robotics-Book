"""Translation service for converting textbook content to Urdu."""

import asyncio
import re
from typing import Dict, Any, Optional
import hashlib
from datetime import datetime
from ..models.translation_cache import TranslationCache, TranslationCacheCreate, TranslationCacheUpdate
from ..models.chapter import Chapter
from ..models.section import Section
from ..config import settings


class TranslationService:
    """Service for translating textbook content to Urdu while preserving technical accuracy."""

    def __init__(self):
        # Check if Urdu translation is enabled
        if not settings.urdu_translation_enabled:
            raise Exception("Urdu translation is disabled in configuration")

        # In a real implementation, this would connect to a translation API or model
        # For this mock, we'll use a simple mapping approach
        self.translation_cache = {}  # Mock cache for demonstration
        self.urdu_translator = UrduTranslator()

    async def translate_content(
        self,
        content_id: str,
        content_type: str,  # 'chapter', 'section', 'text', etc.
        content: str,
        preserve_formatting: bool = True
    ) -> str:
        """
        Translate content to Urdu while preserving technical accuracy.

        Args:
            content_id: Unique identifier for the content
            content_type: Type of content ('chapter', 'section', etc.)
            content: Content to translate
            preserve_formatting: Whether to preserve code, diagrams, and formatting

        Returns:
            Translated content string
        """
        # Check if translation is already cached
        content_hash = self._hash_content(content)
        cache_key = f"{content_id}:{content_type}:{content_hash}"

        if cache_key in self.translation_cache:
            cached_translation = self.translation_cache[cache_key]
            return cached_translation.urdu_translation

        # Perform translation
        translated_content = await self.urdu_translator.translate(
            content,
            preserve_formatting=preserve_formatting
        )

        # Cache the translation
        translation_cache_item = TranslationCacheCreate(
            content_id=content_id,
            content_type=content_type,
            original_content_hash=content_hash,
            urdu_translation=translated_content
        )

        # Create a new cache entry
        cache_entry = TranslationCache(
            id=f"cache_{len(self.translation_cache)}",
            content_id=translation_cache_item.content_id,
            content_type=translation_cache_item.content_type,
            original_content_hash=translation_cache_item.original_content_hash,
            urdu_translation=translation_cache_item.urdu_translation,
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )

        self.translation_cache[cache_key] = cache_entry

        return translated_content

    def _hash_content(self, content: str) -> str:
        """Generate hash for content to detect changes."""
        return hashlib.sha256(content.encode('utf-8')).hexdigest()

    async def update_translation_cache(
        self,
        content_id: str,
        content_type: str,
        new_content: str
    ) -> str:
        """
        Update translation cache when content changes.

        Args:
            content_id: Unique identifier for the content
            content_type: Type of content ('chapter', 'section', etc.)
            new_content: New content to translate

        Returns:
            Updated translated content
        """
        content_hash = self._hash_content(new_content)
        cache_key = f"{content_id}:{content_type}:{content_hash}"

        # Remove old cache entries for this content
        keys_to_remove = []
        for key in self.translation_cache:
            if key.startswith(f"{content_id}:{content_type}:"):
                keys_to_remove.append(key)

        for key in keys_to_remove:
            del self.translation_cache[key]

        # Create new translation
        return await self.translate_content(content_id, content_type, new_content)

    async def translate_chapter(self, chapter: Chapter) -> Dict[str, Any]:
        """
        Translate a complete chapter to Urdu.

        Args:
            chapter: Chapter object to translate

        Returns:
            Dictionary with translated chapter content
        """
        translated_chapter = {
            "id": chapter.id,
            "module_id": chapter.module_id,
            "title": await self.urdu_translator.translate(chapter.title),
            "order": chapter.order,
            "outcomes": await self.urdu_translator.translate(chapter.outcomes) if chapter.outcomes else "",
            "content": await self.urdu_translator.translate(chapter.content) if chapter.content else "",
            "exercises": [],
            "code_examples": chapter.code_examples,  # Preserve code examples without translation
            "diagrams": chapter.diagrams,  # Preserve diagrams without translation
            "created_at": chapter.created_at,
            "updated_at": chapter.updated_at
        }

        # Translate exercises while preserving code and technical terms
        if chapter.exercises:
            for exercise in chapter.exercises:
                translated_exercise = {
                    "question": await self.urdu_translator.translate(exercise.get("question", "")),
                    "answer": await self.urdu_translator.translate(exercise.get("answer", "")),
                    "code": exercise.get("code", ""),  # Preserve code
                    "hints": [
                        await self.urdu_translator.translate(hint) for hint in exercise.get("hints", [])
                    ] if exercise.get("hints") else []
                }
                translated_chapter["exercises"].append(translated_exercise)

        return translated_chapter

    async def translate_section(self, section: Section) -> Dict[str, Any]:
        """
        Translate a section to Urdu.

        Args:
            section: Section object to translate

        Returns:
            Dictionary with translated section content
        """
        translated_section = {
            "id": section.id,
            "chapter_id": section.chapter_id,
            "title": await self.urdu_translator.translate(section.title),
            "order": section.order,
            "content": await self.urdu_translator.translate(section.content),
            "created_at": section.created_at,
            "updated_at": section.updated_at
        }

        return translated_section


class UrduTranslator:
    """Translator specifically for Urdu language that maintains technical accuracy."""

    def __init__(self):
        # In a real implementation, this would connect to a translation API
        # For this mock, we'll use a simple dictionary and rules
        self.technical_terms_map = {
            # Robotics and AI terms
            "robotics": "روبوٹکس",
            "artificial intelligence": "مصنوعی ذہانت",
            "machine learning": "مشین لرننگ",
            "deep learning": "گہرائی سیکھنا",
            "neural network": "عصبی نیٹ ورک",
            "algorithm": "الگورتھم",
            "data": "ڈیٹا",
            "model": "ماڈل",
            "training": "تربیت",
            "inference": "استنتاج",
            "tensor": "ٹینسر",
            "framework": "فریم ورک",

            # ROS2 specific terms
            "ros": "ROS",
            "ros2": "ROS2",
            "node": "نود",
            "topic": "ٹاپک",
            "service": "سروس",
            "action": "ایکشن",
            "parameter": "پیرامیٹر",
            "launch": "لاؤنچ",
            "package": "پیکج",
            "workspace": "ورک اسپیس",
            "msg": "MSG",
            "srv": "SRV",
            "action": "ایکشن",

            # Programming terms
            "python": "Python",
            "javascript": "JavaScript",
            "typescript": "TypeScript",
            "function": "فنکشن",
            "class": "کلاس",
            "object": "آبجیکٹ",
            "method": "میتھڈ",
            "variable": "متغیر",
            "loop": "لوپ",
            "condition": "شرط",
            "array": "اررے",
            "dictionary": "ڈکشنری",
            "list": "فہرست",
            "string": "سٹرنگ",
            "integer": "عدد",
            "boolean": "بولین",
            "true": "سچ",
            "false": "غلط",

            # Isaac specific terms
            "isaac": "Isaac",
            "sim": "Sim",
            "physics": "طبیعیات",
            "rendering": "رینڈر کرنا",
            "simulation": "تقلید",

            # Math and science terms
            "equation": "مساوات",
            "matrix": "میٹرکس",
            "vector": "ویکٹر",
            "derivative": "مشتق",
            "integral": "انٹیگرل",
            "probability": "احتمال",
            "statistics": "شماریات",

            # General academic terms
            "chapter": "باب",
            "section": "سیکشن",
            "exercise": "مشق",
            "solution": "حل",
            "example": "مثال",
            "diagram": "ڈائریگرام",
            "code": "کوڈ",
            "syntax": "Syntax",
            "error": "غلطی",
            "debug": "ڈیبگ",
            "compile": "کمپائل",
            "runtime": "رَن ٹائم",
            "library": "لائبریری",
            "dependency": "انحصار",
            "documentation": "دستاویزات",
            "tutorial": "سیکھنے کا طریقہ",
            "reference": "حوالہ",
            "citation": "حوالہ",
            "bibliography": "کتب خانہ",
            "abstract": "خلاصہ",
            "introduction": "تعارف",
            "conclusion": "خاتمہ",
            "methodology": "طریقہ کار",
            "results": "نتائج",
            "discussion": "بحث",
            "acknowledgment": "تشکر",
            "footnote": "ذیلی نوٹ",
            "appendix": "ضمیمہ",
        }

        # Common English to Urdu translations
        self.common_translations = {
            "the": "کا/کی/کے",
            "a": "ایک",
            "an": "ایک",
            "and": "اور",
            "or": "یا",
            "but": "لیکن",
            "if": "اگر",
            "then": "تو",
            "else": "ورنہ",
            "for": "کے لیے",
            "to": "کو",
            "of": "کا",
            "in": "میں",
            "on": "پر",
            "at": "پر",
            "by": "کی طرف",
            "with": "کے ساتھ",
            "as": "کے طور پر",
            "be": "ہونا",
            "been": "رہا ہے",
            "have": "رکھتا ہے",
            "has": "رکھتا ہے",
            "do": "کرنا",
            "does": "کرتا ہے",
            "will": "کرے گا",
            "would": "کرتا",
            "could": "کر سکتا ہے",
            "should": "چاہیے",
            "may": "شاید",
            "might": "شاید",
            "must": "ضرور",
            "shall": "چاہیے",
            "this": "یہ",
            "that": "وہ",
            "these": "یہ",
            "those": "وہ",
            "i": "میں",
            "you": "آپ",
            "he": "وہ",
            "she": "وہ",
            "it": "وہ",
            "we": "ہم",
            "they": "وہ",
            "me": "مجھے",
            "him": "اسے",
            "her": "اسے",
            "us": "ہمیں",
            "them": "انہیں",
            "my": "میرا",
            "your": "آپ کا",
            "his": "اس کا",
            "her": "اس کا",
            "its": "اس کا",
            "our": "ہمارا",
            "their": "ان کا",
            "what": "کیا",
            "where": "کہاں",
            "when": "کب",
            "why": "کیوں",
            "how": "کیسے",
            "who": "کون",
            "which": "کون سا",
            "all": "سب",
            "some": "کچھ",
            "any": "کوئی",
            "no": "نہیں",
            "not": "نہیں",
            "yes": "جی ہاں",
            "no": "نہیں",
            "here": "یہاں",
            "there": "وہاں",
            "up": "اوپر",
            "down": "نیچے",
            "left": "بائیں",
            "right": "دائیں",
            "before": "پہلے",
            "after": "بعد",
            "above": "اوپر",
            "below": "نیچے",
            "between": "درمیان",
            "through": "ذریعے",
            "during": "کے دوران",
            "until": "تک",
            "against": "کے خلاف",
            "among": "میں سے",
            "without": "بغیر",
            "about": "کے بارے میں",
            "like": "جیسے",
            "throughout": "بھر میں",
            "upon": "پر",
            "since": "سے",
            "except": "کے علاوہ",
            "off": "بند",
            "over": "اوپر",
            "under": "نیچے",
            "again": "دوبارہ",
            "further": "مزید",
            "then": "پھر",
            "once": "ایک بار",
            "here": "یہاں",
            "when": "جب",
            "where": "کہاں",
            "why": "کیوں",
            "how": "کیسے",
            "all": "سب",
            "any": "کوئی",
            "both": "دونوں",
            "each": "ہر ایک",
            "few": "چند",
            "more": "مزید",
            "most": "زیادہ تر",
            "other": "دیگر",
            "some": "کچھ",
            "such": "سے",
            "no": "نہیں",
            "nor": "نہ",
            "not": "نہیں",
            "only": "صرف",
            "own": "اپنا",
            "same": "ویسا",
            "so": "تو",
            "than": "سے",
            "too": "بھی",
            "very": "بہت",
            "can": "کر سکتا ہے",
            "will": "کرے گا",
            "just": "صرف",
            "don't": "نہیں کرتا",
            "good": "اچھا",
            "new": "نیا",
            "first": "پہلا",
            "last": "آخری",
            "long": "لمبا",
            "great": "عظیم",
            "little": "چھوٹا",
            "own": "اپنا",
            "other": "دوسرا",
            "old": "پرانا",
            "right": "صحیح",
            "big": "بڑا",
            "high": "اونچا",
            "different": "مختلف",
            "small": "چھوٹا",
            "large": "بڑا",
            "next": "اگلا",
            "early": "جلد",
            "young": "نوجوان",
            "important": "اہم",
            "few": "چند",
            "public": "عوامی",
            "able": "قابل",
            "play": "کھیلنا",
            "water": "پانی",
            "call": "کال",
            "first": "پہلا",
            "way": "راستہ",
            "day": "دن",
            "man": "آدمی",
            "find": "تلاش کرنا",
            "give": "دینا",
            "work": "کام",
            "many": "کئی",
            "over": "اوپر",
            "know": "جاننا",
            "need": "ضرورت",
            "feel": "محسوس کرنا",
            "seem": "لگتا ہے",
            "ask": "پوچھنا",
            "show": "دکھانا",
            "try": "کوشش کرنا",
            "leave": "چھوڑنا",
            "put": "رکھنا",
            "mean": "مراد",
            "keep": "رکھنا",
            "let": "اجازت دینا",
            "begin": "شروع کرنا",
            "life": "زندگی",
            "always": "ہمیشہ",
            "between": "درمیان",
            "live": "رہنا",
            "hear": "سننا",
        }

    async def translate(self, text: str, preserve_formatting: bool = True) -> str:
        """
        Translate English text to Urdu.

        Args:
            text: English text to translate
            preserve_formatting: Whether to preserve code blocks, etc.

        Returns:
            Translated Urdu text
        """
        # Simulate translation API call delay
        await asyncio.sleep(0.1)

        if not text:
            return ""

        # If preserving formatting, extract code blocks and other special content
        if preserve_formatting:
            # Extract code blocks (enclosed in triple backticks)
            code_blocks = []
            code_pattern = r'```.*?```'
            code_matches = re.findall(code_pattern, text, re.DOTALL)
            for i, match in enumerate(code_matches):
                placeholder = f"[[CODE_BLOCK_{i}]]"
                text = text.replace(match, placeholder, 1)
                code_blocks.append(match)

            # Extract inline code (enclosed in single backticks)
            inline_codes = []
            inline_code_pattern = r'`[^`]*`'
            inline_code_matches = re.findall(inline_code_pattern, text)
            for i, match in enumerate(inline_code_matches):
                placeholder = f"[[INLINE_CODE_{i}]]"
                text = text.replace(match, placeholder, 1)
                inline_codes.append(match)

            # Extract links
            links = []
            link_pattern = r'\[([^\]]+)\]\([^)]+\)'
            link_matches = re.findall(link_pattern, text)
            for i, match in enumerate(link_matches):
                placeholder = f"[[LINK_{i}]]"
                text = re.sub(link_pattern, placeholder, text, count=1)
                links.append(match)

        # Split text into sentences/paragraphs to preserve structure
        paragraphs = text.split('\n\n')
        translated_paragraphs = []

        for paragraph in paragraphs:
            sentences = paragraph.split('. ')
            translated_sentences = []

            for sentence in sentences:
                # Clean up the sentence
                sentence = sentence.strip()
                if not sentence:
                    continue

                # Translate the sentence
                translated_sentence = self._translate_sentence(sentence)
                translated_sentences.append(translated_sentence)

            translated_paragraph = '. '.join(translated_sentences)
            translated_paragraphs.append(translated_paragraph)

        result = '\n\n'.join(translated_paragraphs)

        # Restore preserved elements
        if preserve_formatting:
            for i, code_block in enumerate(code_blocks):
                placeholder = f"[[CODE_BLOCK_{i}]]"
                result = result.replace(placeholder, code_block)

            for i, inline_code in enumerate(inline_codes):
                placeholder = f"[[INLINE_CODE_{i}]]"
                result = result.replace(placeholder, inline_code)

            for i, link in enumerate(links):
                placeholder = f"[[LINK_{i}]]"
                result = result.replace(placeholder, link)

        return result

    def _translate_sentence(self, sentence: str) -> str:
        """Translate a single sentence from English to Urdu."""
        # For this mock implementation, we'll do a simple word-for-word translation
        # with special handling for technical terms

        # First, handle technical terms (longer terms first to avoid partial matches)
        sorted_terms = sorted(self.technical_terms_map.keys(), key=len, reverse=True)
        translated_sentence = sentence

        for eng_term in sorted_terms:
            if eng_term.lower() in translated_sentence.lower():
                # Replace while preserving capitalization
                translated_sentence = re.sub(
                    r'\b' + re.escape(eng_term) + r'\b',
                    self.technical_terms_map[eng_term],
                    translated_sentence,
                    flags=re.IGNORECASE
                )

        # Then handle common words (shorter words first to avoid conflicts)
        sorted_common = sorted(self.common_translations.keys(), key=len, reverse=True)
        for eng_word in sorted_common:
            if eng_word.lower() in translated_sentence.lower():
                # Replace while preserving capitalization
                translated_sentence = re.sub(
                    r'\b' + re.escape(eng_word) + r'\b',
                    self.common_translations[eng_word],
                    translated_sentence,
                    flags=re.IGNORECASE
                )

        # For any remaining English words, we'll keep them in English
        # (In a real implementation, we'd use a proper translation API)
        return translated_sentence


# Import needed for the regex patterns used above
import re