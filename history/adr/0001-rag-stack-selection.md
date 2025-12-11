# ADR-0001: RAG Stack Selection

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** 001-textbook-rag-platform
- **Context:** The Physical AI & Humanoid Robotics textbook platform requires a Retrieval Augmented Generation (RAG) system that provides book-only answers with citations and confidence scores in under 800ms. The RAG stack must align with the project constitution which mandates the specific technology stack.

## Decision

Use the integrated RAG stack: FastAPI + Qdrant + NeonDB + ChatKit + OpenAI Agents. This decision is mandated by the project constitution and engineering standards.

- **Backend Framework**: FastAPI (Python 3.11)
- **Vector Database**: Qdrant (for textbook content embeddings)
- **Relational Database**: NeonDB (PostgreSQL) for user data and personalization
- **Chat Framework**: ChatKit (for conversation management)
- **AI Provider**: OpenAI Agents (for content generation and reasoning)

## Consequences

### Positive

- Alignment with project constitution and engineering standards
- Performance target of <800ms response time is achievable with this stack
- Book-only content grounding with proper citation capabilities
- Integrated solution with established patterns and documentation
- Confidence scoring and selected-text answering functionality supported
- Compliance with quality-first non-negotiable principle

### Negative

- Vendor lock-in to specific technology ecosystem
- Potential complexity in managing multiple database systems (PostgreSQL + Vector DB)
- Dependency on OpenAI for core functionality
- Learning curve for team members unfamiliar with the stack

## Alternatives Considered

Alternative Stack: LangChain + Pinecone + Supabase + custom API
- Rejected because: Would violate RAG Architecture Standard principle in constitution which mandates the specific FastAPI + Qdrant + NeonDB + ChatKit + OpenAI Agents stack

Alternative Stack: Custom solution with different vector database
- Rejected because: Would not align with the established RAG Architecture Standard required by the project constitution

## References

- Feature Spec: specs/001-textbook-rag-platform/spec.md
- Implementation Plan: specs/001-textbook-rag-platform/plan.md
- Related ADRs: ADR-0004 (Data Storage Strategy) - relates to NeonDB component
- Evaluator Evidence: specs/001-textbook-rag-platform/research.md#11-rag-stack-selection
