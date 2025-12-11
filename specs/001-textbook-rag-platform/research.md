# Research: Physical AI & Humanoid Robotics Textbook Platform

## 1. Technology Stack Decisions

### 1.1 RAG Stack Selection
**Decision**: FastAPI + Qdrant + NeonDB + ChatKit + OpenAI Agents
**Rationale**: This stack is mandated by the project constitution and engineering standards. It provides the required performance (<800ms response time), book-only content grounding, and proper citation capabilities.
**Alternatives considered**:
- Alternative: LangChain + Pinecone + Supabase + custom API
- Rejected because: Would violate RAG Architecture Standard principle in constitution

### 1.2 Frontend Framework
**Decision**: Docusaurus
**Rationale**: Docusaurus is specifically designed for documentation sites and provides excellent support for structured content like textbooks with built-in search, versioning, and theming capabilities.
**Alternatives considered**:
- Alternative: Custom React + Next.js application
- Rejected because: Would require significant additional work for documentation features that Docusaurus provides out-of-the-box

### 1.3 Authentication System
**Decision**: Better-Auth
**Rationale**: Better-Auth is lightweight, supports storing user background information for personalization, and integrates well with modern web applications.
**Alternatives considered**:
- Alternative: Auth0, Firebase Auth, Supabase Auth
- Rejected because: Would introduce unnecessary complexity for a platform that primarily needs user background storage for personalization

## 2. Architecture Patterns

### 2.1 Subagent Architecture
**Decision**: Implement modular subagents (Research, Writing, Robotics, ROS2, Gazebo/Unity, Isaac, RAG, Frontend, Auth)
**Rationale**: Aligns with the Modular Subagents & Skills principle in the constitution, enabling reusable, independently testable components.
**Alternatives considered**:
- Alternative: Monolithic processing approach
- Rejected because: Would violate the Modular Subagents & Skills principle

### 2.2 Content Structure
**Decision**: 5 modules (ROS2, Gazebo/Unity, Isaac, VLA, Capstone) with 2-3 chapters per module and 3-7 sections per chapter
**Rationale**: Mandated by the book structure requirements in both the specification and constitution.
**Alternatives considered**:
- Alternative: Different module organization or different chapter/section counts
- Rejected because: Would violate the Book Structure principle in constitution

## 3. Performance & Quality Requirements

### 3.1 RAG Response Time
**Decision**: <800ms response time for 95% of queries
**Rationale**: Required by the constitution and engineering standards to ensure responsive user experience
**Alternatives considered**:
- Alternative: Slower response times (1-2 seconds)
- Rejected because: Would violate Quality-First Non-Negotiable principle

### 3.2 Content Quality Standards
**Decision**: Grade 10-12 level content with ≥1 diagram + ≥2 exercises per chapter, APA citations, peer-reviewed sources
**Rationale**: Required by both the specification and constitution to maintain educational quality
**Alternatives considered**:
- Alternative: Lower grade level or fewer diagrams/exercises
- Rejected because: Would violate Quality-First Non-Negotiable principle

## 4. Safety & Compliance

### 4.1 Safety-First Design
**Decision**: Include safety warnings for all physical robotics steps and avoid unsupported hardware recommendations
**Rationale**: Required by the Safety-First Design principle in the constitution
**Alternatives considered**:
- Alternative: Include all robotics procedures regardless of safety or hardware support
- Rejected because: Would violate Safety-First Design principle

## 5. Internationalization

### 5.1 Urdu Translation
**Decision**: Implement backend route /api/translate-ur with translation service
**Rationale**: Required by Multi-Language & Personalization Support principle in constitution
**Alternatives considered**:
- Alternative: No translation support
- Rejected because: Would violate Multi-Language & Personalization Support principle

## 6. Deployment Architecture

### 6.1 Deployment Strategy
**Decision**: Docusaurus → GitHub Pages/Vercel, FastAPI → Railway/Render/Fly.io, Qdrant Cloud + NeonDB
**Rationale**: Mandated by deployment standards in constitution
**Alternatives considered**:
- Alternative: Different hosting platforms
- Rejected because: Would violate Deployment Standards principle