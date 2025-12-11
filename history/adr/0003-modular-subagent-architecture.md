# ADR-0003: Modular Subagent Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** 001-textbook-rag-platform
- **Context:** The Physical AI & Humanoid Robotics textbook platform requires specialized processing for different aspects of the system (research, writing, robotics, RAG, frontend, auth). The project constitution mandates the use of modular subagents and reusable skills to ensure components are reusable and independently testable.

## Decision

Implement a modular subagent architecture with specialized agents for different domains. This aligns with the Modular Subagents & Skills principle in the project constitution.

- **Research Agent**: Handles fact gathering, outlines, and citations
- **Writing Agent**: Handles chapter creation, diagrams, and rewriting
- **Robotics Agent**: Handles ROS2/Isaac/Gazebo code generation
- **RAG Agent**: Handles content ingestion and retrieval QA
- **Frontend Agent**: Handles website UI and components
- **Auth Agent**: Handles authentication and user management
- **Required Skills**: fact-check, citations, ros2-launch, gazebo-world, rag-ingest, personalize-content

## Consequences

### Positive

- Alignment with project constitution's Modular Subagents & Skills principle
- Components are reusable and independently testable
- Clear separation of concerns between different domains
- Scalable architecture that allows for specialization
- Skills can be modular and documented for reuse
- Easier maintenance and updates to specific components
- Supports the research → foundation → analysis → synthesis workflow

### Negative

- Increased complexity in coordinating between different agents
- Potential overhead in inter-agent communication
- More complex initial setup compared to monolithic approach
- Requires careful design of agent interfaces and contracts
- Learning curve for team members unfamiliar with agent-based architectures

## Alternatives Considered

Alternative: Monolithic processing approach
- Rejected because: Would violate the Modular Subagents & Skills principle in the constitution and reduce reusability and testability

Alternative: Service-oriented architecture without specialized agents
- Rejected because: Would not align with the specific subagent pattern required by the project constitution

## References

- Feature Spec: specs/001-textbook-rag-platform/spec.md
- Implementation Plan: specs/001-textbook-rag-platform/plan.md
- Evaluator Evidence: specs/001-textbook-rag-platform/research.md#21-subagent-architecture
