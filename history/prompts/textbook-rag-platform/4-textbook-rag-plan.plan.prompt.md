---
id: 4
title: Textbook RAG Platform Plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-rag-platform
branch: 001-textbook-rag-platform
user: user
command: /sp.plan
labels: [plan, textbook, rag, ai, robotics, architecture]
links:
  spec: specs/001-textbook-rag-platform/spec.md
  plan: specs/001-textbook-rag-platform/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-textbook-rag-platform/plan.md
 - specs/001-textbook-rag-platform/research.md
 - specs/001-textbook-rag-platform/data-model.md
 - specs/001-textbook-rag-platform/quickstart.md
 - specs/001-textbook-rag-platform/contracts/rag-query-contract.md
tests:
 - none
---

## Prompt

```text
# Physical AI & Humanoid Robotics — Master Plan

## 0. Inputs
Follow: (a) Constitution, (b) /sp.specify standard.

## 1. High-Level Phases
1. Research → 2. Book Writing → 3. Code & Simulation →
4. RAG Backend → 5. Website → 6. Auth & Personalization →
7. Urdu Translation → 8. Testing → 9. Deployment → 10. Demo.

## 2. Task Breakdown

### 2.1 Research Phase
- Gather authoritative sources (ROS2, Gazebo, Isaac).
- Produce structured outlines for 5 modules.
- Output: Finalized textbook outline (modules → chapters → sections).

### 2.2 Textbook Writing Phase
- Write each chapter with outcomes, explanations, Mermaid diagram, code, exercises.
- Use APA citations + fact-check.skill.
- Subagents: Research, AcademicWriting, Robotics.

### 2.3 Code & Simulation Assets
- Create ROS2 nodes, launch files.
- Build Gazebo/Unity world templates.
- Add Isaac sample pipelines.
- Skills: ros2-launch.skill, gazebo-world.skill.
- Deliverable: runnable code folder.

### 2.4 RAG Backend
- FastAPI skeleton.
- Qdrant ingestion (rag-ingest.skill).
- NeonDB schema (users, backgrounds, personalization).
- ChatKit + OpenAI Agents integration.
- Requirements: citations, confidence, book-only grounding.

### 2.5 Docusaurus Website
- Install + theme.
- Generate pages for all modules/chapters.
- Add interactive buttons:
  - /api/personalize
  - /api/translate-ur
- Subagent: Frontend/UI.

### 2.6 Auth & Personalization
- Implement Better-Auth.
- Capture: user background, level, role.
- Personalize chapters via personalize-content.skill.
- Output: dynamic HTML/json injected into chapter pages.

### 2.7 Urdu Translation
- Backend route /api/translate-ur.
- Translate chapter chunks using agent.
- Maintain diagrams/code unmodified.
- Output: mirrored Urdu pages.

### 2.8 Integration Testing
- Validate textbook content structure.
- Validate ROS2/Isaac scripts run error-free.
- Validate RAG latency (<800ms), citations, confidence.
- Validate Urdu + personalization flows.

### 2.9 Deployment
- Docusaurus → GitHub Pages / Vercel.
- FastAPI → Railway / Render / Fly.io.
- Qdrant Cloud + NeonDB.

### 2.10 Demo Preparation
- 90-second video script:
  textbook → RAG → personalization → Urdu translation.
- Final GitHub repo packaging.

## 3. Milestones
- M1: Book outline complete.
- M2: 5 modules fully drafted.
- M3: ROS2/Gazebo/Isaac code validated.
- M4: RAG backend functional.
- M5: Website deployed.
- M6: Personalization + Urdu stable.
- M7: Final integration + demo.

## 4. Agents & Skills Map
- ResearchAgent → facts, outlines, citations.
- WritingAgent → chapters, diagrams, rewriting.
- RoboticsAgent → ROS2/Isaac/Gazebo code.
- RAGAgent → ingestion, retrieval QA.
- FrontendAgent → website UI.
- AuthAgent → Better-Auth setup.

Skills: fact-check, citations, ros2-launch, gazebo-world, rag-ingest, personalize-content.

## 5. Acceptance Criteria
- Book: accurate, APA, 1 diagram/chapter, 2 exercises, grade 10–12.
- Code: error-free, tested.
- RAG: book-only answers, citations, confidence, <800ms.
- Website: deploys cleanly.
- Personalization & Urdu: functional.
```

## Response snapshot

Completed comprehensive implementation plan for the Physical AI & Humanoid Robotics Textbook Platform. The plan includes:

- Detailed technical context with specific technologies (Python 3.11, FastAPI, Qdrant, NeonDB, Docusaurus, etc.)
- Constitution check validation confirming all principles are satisfied
- Multi-tier project structure with separate backend, frontend, and robotics directories
- Research document covering technology stack decisions and architecture patterns
- Complete data model with 8 core entities and validation rules
- Quickstart guide for developers
- API contract for the RAG query endpoint
- Agent context updated with new technologies

All constitution gates passed, confirming the plan aligns with the Physical AI & Humanoid Robotics project principles.

## Outcome

- ✅ Impact: Complete implementation plan for textbook platform created
- 🧪 Tests: none
- 📁 Files: specs/001-textbook-rag-platform/plan.md, research.md, data-model.md, quickstart.md, contracts/rag-query-contract.md
- 🔁 Next prompts: /sp.adr to create architecture decision records
- 🧠 Reflection: Plan comprehensively covers all required modules and functionality

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none