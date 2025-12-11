<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: [PRINCIPLE_1_NAME] → Physical AI & Humanoid Robotics Focus, [PRINCIPLE_2_NAME] → RAG Architecture Standard, [PRINCIPLE_3_NAME] → Quality-First Non-Negotiable, [PRINCIPLE_4_NAME] → Modular Subagents & Skills, [PRINCIPLE_5_NAME] → Safety-First Design
- Added sections: Engineering Standards, Development Workflow
- Removed sections: None
- Templates requiring updates: ✅ .specify/templates/plan-template.md, ✅ .specify/templates/spec-template.md, ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### Physical AI & Humanoid Robotics Focus
Every feature and component must advance the mission of creating a comprehensive Physical AI & Humanoid Robotics textbook and platform. All development efforts must directly support the five core modules: ROS2, Gazebo/Unity, Isaac, VLA, and Capstone. No generic libraries or organizational-only components that don't serve the robotics education mission.

### RAG Architecture Standard
All AI/ML functionality must leverage the established RAG stack: FastAPI + Qdrant + NeonDB + ChatKit + OpenAI Agents. Every AI interaction must cite book-only content, include confidence scores, and support text selection for verification. The RAG system must achieve <800ms response time with proper citations.

### Quality-First Non-Negotiable
All content and code must meet the highest standards: Accurate, cited, structured content with ≥1 diagram + ≥2 exercises per chapter at grade 10-12 level; Error-free, valid syntax, commented, tested code; Book-only answers from RAG with citations + confidence. No hallucinations in content or AI responses.

### Modular Subagents & Skills
Development must follow the modular subagent pattern: Research, Writing, Robotics, ROS2, Gazebo/Unity, Isaac, RAG, Frontend, Auth subagents with modular, documented skills (fact-check, citations, ros2-launch, gazebo-world, rag-ingest, personalize-content). Each component must be reusable and independently testable.

### Safety-First Design
All robotics steps must be safe and supported by hardware. No unsafe robotics procedures or unsupported hardware recommendations. Clear warnings must be provided for any physical risks. All implementations must prioritize safety over functionality.

### Multi-Language & Personalization Support
All content and interfaces must support personalization and Urdu translation. The system must provide personalization via /api/personalize and Urdu translation via /api/translate-ur. User backgrounds must be stored to personalize chapters appropriately.

## Engineering Standards

### RAG Implementation
FastAPI + Qdrant + NeonDB + ChatKit + Agents stack is mandatory. Cite book-only content; include confidence; support text selection. Response time must be under 800ms with proper citations and confidence scores.

### Website Requirements
Website must include Personalize and Urdu translation buttons that connect to /api/personalize and /api/translate-ur endpoints respectively. All user backgrounds must be stored to enable chapter personalization.

### Authentication & Data Management
Authentication system must store user background information and use it to personalize chapters. All user data must be handled according to privacy standards with proper security measures.

## Development Workflow

### Writing Standards
All content must be Clear, correct, concise; use APA citations; be fact-checked; and avoid hallucinations. Each chapter must include: outcomes, explanations, Mermaid diagram, ROS2/Gazebo/Isaac code, and exercises.

### Book Structure
The textbook follows a strict structure: Five modules (ROS2, Gazebo/Unity, Isaac, VLA, Capstone), each with 2-3 chapters, each chapter with 3-7 sections. All content must fit within this framework.

### Research Methodology
Follow Research → Foundation → Analysis → Synthesis methodology. Use inline research and /sp.rewrite enforce="constitution" to ensure compliance with all standards.

### Deployment Standards
Docusaurus → GitHub Pages/Vercel for website. FastAPI → Railway/Render/Fly.io for API. Qdrant Cloud + NeonDB for data storage. All deployments must follow these standards.

## Governance

This constitution governs all development activities for the Physical AI & Humanoid Robotics project. All code, content, and features must comply with these principles. Any deviation requires explicit approval and documentation of the architectural decision. All pull requests and reviews must verify compliance with these standards. All development must follow the Research → Foundation → Analysis → Synthesis workflow and use inline research with constitution enforcement.

**Version**: 1.1.0 | **Ratified**: 2025-01-15 | **Last Amended**: 2025-12-10
