---
id: 5
title: Textbook RAG Platform ADRs
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-rag-platform
branch: 001-textbook-rag-platform
user: user
command: /sp.adr
labels: [adr, architecture, textbook, rag, ai, robotics]
links:
  spec: specs/001-textbook-rag-platform/spec.md
  plan: specs/001-textbook-rag-platform/plan.md
  adr_0001: history/adr/0001-rag-stack-selection.md
  adr_0002: history/adr/0002-frontend-framework-selection.md
  adr_0003: history/adr/0003-modular-subagent-architecture.md
  adr_0004: history/adr/0004-data-storage-strategy.md
  adr_0005: history/adr/0005-deployment-architecture.md
  ticket: null
  pr: null
files:
 - history/adr/0001-rag-stack-selection.md
 - history/adr/0002-frontend-framework-selection.md
 - history/adr/0003-modular-subagent-architecture.md
 - history/adr/0004-data-storage-strategy.md
 - history/adr/0005-deployment-architecture.md
tests:
 - none
---

## Prompt

```text
# Physical AI & Humanoid Robotics — ADR guidance

## Purpose
Review the current plan and record the project's key architectural decisions in ADRs.
Focus on decisions that:
1) Affect multiple sections or phases.
2) Had meaningful alternatives.
3) Will shape how we write the book or build the system.
4) Someone might question later during review or maintenance.

## ADR format
Use a concise, standard ADR style. Include at minimum:
- Title
- Status
- Context
- Decision
- Consequences

This aligns with widely used ADR guidance (e.g., Nygard‑style ADRs). :contentReference[oaicite:0]{index=0}

## Trigger points for creating ADRs
Create a new ADR when any of these occur:
1) Selecting or rejecting a major technology or architecture pattern.
2) Defining the RAG stack details, performance targets, or citation rules.
3) Choosing deployment platforms or data stores.
4) Setting personalization, translation, or safety constraints.
5) Deciding on modular subagent boundaries or required skills.
6) Altering the book structure, standards, or workflow in a way that impacts multiple modules.

## Content guidance per ADR
Each ADR should clearly state:
- Why the decision matters to the project's Constitution / standard / plan.
- What other options were considered and why they were not chosen.
- Expected benefits and tradeoffs, including risks or future costs.
- Any actions needed to implement or monitor the decision.

## Output
Produce ADRs as short markdown entries, one per decision.
Reference the plan's phases, modules, or standards where relevant.
Ensure each ADR is traceable to the Constitution or plan to support future reviews, governance, or audits.
```

## Response snapshot

Created 5 Architecture Decision Records (ADRs) for the Physical AI & Humanoid Robotics Textbook Platform:

1. ADR-0001: RAG Stack Selection - Documenting the decision to use FastAPI + Qdrant + NeonDB + ChatKit + OpenAI Agents as mandated by the project constitution
2. ADR-0002: Frontend Framework Selection - Documenting the decision to use Docusaurus for the textbook platform
3. ADR-0003: Modular Subagent Architecture - Documenting the decision to implement specialized agents for different domains
4. ADR-0004: Data Storage Strategy - Documenting the hybrid approach using NeonDB (PostgreSQL) and Qdrant
5. ADR-0005: Deployment Architecture - Documenting the distributed deployment approach with platform-specific hosting

Each ADR includes context, decision, consequences (positive and negative), alternatives considered, and references to related documents. All ADRs align with the project constitution and engineering standards.

## Outcome

- ✅ Impact: 5 key architectural decisions documented as ADRs
- 🧪 Tests: none
- 📁 Files: history/adr/0001-rag-stack-selection.md, history/adr/0002-frontend-framework-selection.md, history/adr/0003-modular-subagent-architecture.md, history/adr/0004-data-storage-strategy.md, history/adr/0005-deployment-architecture.md
- 🔁 Next prompts: /sp.tasks to generate implementation tasks
- 🧠 Reflection: ADRs provide clear documentation for future reference and governance

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none