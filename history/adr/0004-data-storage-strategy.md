# ADR-0004: Data Storage Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** 001-textbook-rag-platform
- **Context:** The Physical AI & Humanoid Robotics textbook platform requires different types of data storage: structured user data and personalization information, and unstructured textbook content for the RAG system. The system needs to support both relational data operations and vector similarity searches.

## Decision

Implement a hybrid data storage strategy using multiple specialized storage systems:

- **Relational Database**: NeonDB (PostgreSQL) for user data, personalization profiles, user interactions, and structured content metadata
- **Vector Database**: Qdrant for textbook content embeddings to support RAG functionality
- **File Storage**: For static assets like diagrams, code examples, and multimedia content
- **Caching Layer**: For frequently accessed content and translation caching

## Consequences

### Positive

- Optimal storage for different data types (relational for structured data, vector for content similarity)
- High-performance RAG queries with Qdrant's vector search capabilities
- Scalable architecture that can handle both user data and large content repositories
- Support for complex queries on user data with PostgreSQL
- Efficient content retrieval for the RAG system
- Compliance with the <800ms response time requirement

### Negative

- Increased complexity in managing multiple database systems
- More complex deployment and operations
- Potential consistency challenges between different storage systems
- Higher infrastructure costs compared to single-database solutions
- More complex backup and recovery procedures
- Learning curve for team members on multiple database technologies

## Alternatives Considered

Alternative: Single PostgreSQL database for all data
- Rejected because: Would not efficiently support vector similarity searches required for RAG functionality

Alternative: Single vector database for all data
- Rejected because: Would not efficiently support relational operations needed for user data and personalization

Alternative: Document database (MongoDB) for all data
- Rejected because: Would not provide the specialized capabilities needed for either relational operations or vector similarity searches

## References

- Feature Spec: specs/001-textbook-rag-platform/spec.md
- Implementation Plan: specs/001-textbook-rag-platform/plan.md
- Data Model: specs/001-textbook-rag-platform/data-model.md
- Related ADRs: ADR-0001 (RAG Stack Selection) - relates to Qdrant component
- Evaluator Evidence: specs/001-textbook-rag-platform/research.md#technology-stack-decisions
