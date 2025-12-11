# Data Model: Physical AI & Humanoid Robotics Textbook Platform

## 1. Core Entities

### 1.1 User
**Description**: A researcher who accesses the textbook, with background information that influences personalization

**Fields**:
- `id` (string): Unique identifier
- `email` (string): User's email address
- `name` (string): User's full name
- `background` (JSON): User's technical background and expertise level
- `research_interests` (array): Array of research areas of interest
- `created_at` (datetime): Account creation timestamp
- `updated_at` (datetime): Last update timestamp

**Relationships**:
- One-to-many with PersonalizationProfile
- One-to-many with UserInteraction

### 1.2 Module
**Description**: One of the 5 core modules (ROS2, Gazebo/Unity, Isaac, VLA, Capstone)

**Fields**:
- `id` (string): Unique identifier
- `name` (string): Module name (e.g., "ROS2")
- `title` (string): Full title of the module
- `description` (string): Brief description of the module
- `order` (integer): Display order in the textbook
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Relationships**:
- One-to-many with Chapter
- Many-to-many with User (through PersonalizationProfile)

### 1.3 Chapter
**Description**: A chapter within a module containing educational content

**Fields**:
- `id` (string): Unique identifier
- `module_id` (string): Reference to parent module
- `title` (string): Chapter title
- `order` (integer): Chapter order within module (1-3)
- `outcomes` (text): Learning outcomes for the chapter
- `content` (text): Main content body
- `exercises` (JSON): Array of exercises with answers
- `code_examples` (JSON): ROS2/Gazebo/Isaac code examples
- `diagrams` (JSON): Mermaid diagrams and other visuals
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Relationships**:
- Many-to-one with Module
- One-to-many with Section
- One-to-many with UserInteraction

### 1.4 Section
**Description**: A section within a chapter (3-7 per chapter)

**Fields**:
- `id` (string): Unique identifier
- `chapter_id` (string): Reference to parent chapter
- `title` (string): Section title
- `order` (integer): Section order within chapter
- `content` (text): Section content
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Relationships**:
- Many-to-one with Chapter

### 1.5 PersonalizationProfile
**Description**: User background information that influences how textbook content is presented

**Fields**:
- `id` (string): Unique identifier
- `user_id` (string): Reference to user
- `module_id` (string): Reference to module
- `personalization_settings` (JSON): Customized display settings
- `learning_progress` (JSON): Progress tracking data
- `preference_tags` (array): Array of preference tags
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Relationships**:
- Many-to-one with User
- Many-to-one with Module
- One-to-many with PersonalizedContent

### 1.6 RAGResponse
**Description**: AI-generated answers based on textbook content with APA citations, confidence scores, and <800ms response time

**Fields**:
- `id` (string): Unique identifier
- `query` (text): Original user query
- `response` (text): Generated response
- `citations` (JSON): Array of source citations in APA format
- `confidence_score` (float): Confidence level (0.0-1.0)
- `response_time_ms` (integer): Response time in milliseconds
- `book_content_used` (array): IDs of content used in response
- `created_at` (datetime): Creation timestamp

**Relationships**:
- One-to-many with UserInteraction

### 1.7 UserInteraction
**Description**: Record of user interactions with the platform

**Fields**:
- `id` (string): Unique identifier
- `user_id` (string): Reference to user
- `chapter_id` (string): Reference to chapter (if applicable)
- `interaction_type` (string): Type of interaction (view, search, exercise, etc.)
- `content` (text): Interaction content or query
- `result` (text): Result of interaction
- `timestamp` (datetime): When interaction occurred
- `session_id` (string): Session identifier

**Relationships**:
- Many-to-one with User
- Many-to-one with Chapter
- One-to-one with RAGResponse (if applicable)

### 1.8 TranslationCache
**Description**: Cached Urdu translations of textbook content

**Fields**:
- `id` (string): Unique identifier
- `content_id` (string): Reference to original content (chapter/section)
- `content_type` (string): Type of content ('chapter', 'section')
- `original_content_hash` (string): Hash of original content to detect changes
- `urdu_translation` (text): Urdu translation
- `created_at` (datetime): Creation timestamp
- `updated_at` (datetime): Last update timestamp

**Relationships**:
- One-to-one with Chapter or Section (polymorphic)

## 2. Validation Rules

### 2.1 Content Validation
- All content must meet grade 10-12 level educational standards
- Each chapter must include ≥1 diagram and ≥2 exercises
- All code examples must have valid ROS2/Isaac syntax
- Content must include APA citations from peer-reviewed sources only

### 2.2 RAG Validation
- All responses must be based on book-only content
- Responses must include proper APA citations
- Confidence scores must be between 0.0 and 1.0
- Response time must be recorded and monitored

### 2.3 Safety Validation
- All robotics content must include safety warnings
- Hardware requirements must be clearly specified
- Unsupported hardware recommendations are prohibited

## 3. State Transitions

### 3.1 Content States
- `draft` → `reviewed` → `published` → `archived`
- Content in `draft` state is not visible to users
- Content in `reviewed` state is pending final approval
- Content in `published` state is visible to users
- Content in `archived` state is no longer accessible

### 3.2 User Progress States
- `not_started` → `in_progress` → `completed`
- Progress tracking allows users to resume where they left off
- Completion status is stored in PersonalizationProfile