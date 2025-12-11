---
id: 1
title: Physical AI Humanoid Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, physical-ai, humanoid-robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Mission

Create a Physical AI & Humanoid Robotics Docusaurus textbook.

Build a RAG chatbot (OpenAI Agents, ChatKit, FastAPI, Qdrant, NeonDB).

Deploy website with personalization + Urdu translation.

Provide reusable subagents + skills for writing, research, code, robotics.

2. Writing Standards

Clear, correct, concise; APA citations; fact-checked.

Each chapter must include: outcomes, explanations, Mermaid diagram, ROS2/Gazebo/Isaac code, exercises.

No hallucinations.

3. Book Structure

Five modules: ROS2, Gazebo/Unity, Isaac, VLA, Capstone.

Modules: 2–3 chapters each.

Chapters: 3–7 sections.

4. Engineering Standards
4.1 RAG

FastAPI + Qdrant + NeonDB + ChatKit + Agents.

Cite book-only content; include confidence; support text selection.

4.2 Website

Buttons: Personalize → /api/personalize, Urdu → /api/translate-ur.

4.3 Auth

Store user background; personalize chapters.

5. Subagents & Skills

Subagents: Research, Writing, Robotics, ROS2, Gazebo/Unity, Isaac, RAG, Frontend, Auth.

Skills: modular, documented (fact-check, citations, ros2-launch, gazebo-world, rag-ingest, personalize-content).

6. Quality
Content: Accurate, cited, structured; ≥1 diagram + ≥2 exercises; grade 10–12 level.
Code: Error-free, valid syntax, commented, tested.
RAG: Book-only answers; <800 ms; citations + confidence.
7. Workflow

Research → Foundation → Analysis → Synthesis.
Use inline research + /sp.rewrite enforce="constitution".

8. Deployment

Docusaurus → GitHub Pages/Vercel.
FastAPI → Railway/Render/Fly.io.
Qdrant Cloud + NeonDB.

9. Hackathon Optimization

Use subagents, skills, Better-Auth, Urdu translation, RAG citations, full textbook.

10. Safety

No unsafe robotics steps; no unsupported hardware; warn about physical risks.
```

## Response snapshot

Updated the Physical AI & Humanoid Robotics Constitution to reflect the project mission and standards. The new constitution includes:

Core Principles:
- Physical AI & Humanoid Robotics Focus
- RAG Architecture Standard
- Quality-First Non-Negotiable
- Modular Subagents & Skills
- Safety-First Design
- Multi-Language & Personalization Support

Sections:
- Engineering Standards (RAG Implementation, Website Requirements, Authentication & Data Management)
- Development Workflow (Writing Standards, Book Structure, Research Methodology, Deployment Standards)

The constitution now properly governs all development activities for the Physical AI & Humanoid Robotics project with version 1.1.0, ratified 2025-01-15, last amended 2025-12-10.

## Outcome

- ✅ Impact: Updated project constitution to reflect Physical AI & Humanoid Robotics mission
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: Follow constitution for all future development
- 🧠 Reflection: Constitution now properly guides all project activities with clear principles

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none