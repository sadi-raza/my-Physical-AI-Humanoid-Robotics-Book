# ADR-0005: Deployment Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** 001-textbook-rag-platform
- **Context:** The Physical AI & Humanoid Robotics textbook platform consists of multiple components (frontend, backend, databases) that require different deployment strategies and scaling characteristics. The project constitution mandates specific deployment platforms.

## Decision

Implement a distributed deployment architecture with platform-specific hosting:

- **Frontend (Docusaurus)**: Deploy to GitHub Pages or Vercel as mandated by constitution
- **Backend (FastAPI)**: Deploy to Railway, Render, or Fly.io as mandated by constitution
- **Vector Database (Qdrant)**: Deploy to Qdrant Cloud as mandated by constitution
- **Relational Database (NeonDB)**: Deploy to NeonDB cloud service as mandated by constitution
- **CI/CD Pipeline**: Separate pipelines for frontend and backend with appropriate testing

## Consequences

### Positive

- Alignment with project constitution and deployment standards
- Optimal platform choice for each component's specific requirements
- Scalable architecture with independent scaling of frontend and backend
- Cost-effective as each platform is optimized for its specific workload
- Managed services reduce operational overhead
- Leverage platform-specific features and optimizations

### Negative

- Increased complexity in managing multiple deployment platforms
- Potential vendor lock-in to specific cloud providers
- More complex monitoring and observability across platforms
- Cross-platform debugging and troubleshooting challenges
- Dependency on multiple external services
- Potentially higher costs compared to single-platform solutions

## Alternatives Considered

Alternative: Single cloud platform (e.g., AWS, GCP, Azure) for all components
- Rejected because: Would violate the Deployment Standards principle in constitution which mandates specific platforms for each component

Alternative: Self-hosted infrastructure
- Rejected because: Would not align with the cloud-first approach required by the project constitution and would increase operational complexity

## References

- Feature Spec: specs/001-textbook-rag-platform/spec.md
- Implementation Plan: specs/001-textbook-rag-platform/plan.md
- Evaluator Evidence: specs/001-textbook-rag-platform/research.md#61-deployment-strategy
