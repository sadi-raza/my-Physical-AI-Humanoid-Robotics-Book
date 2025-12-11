# RAG Query API Contract

## Endpoint
`POST /api/rag/query`

## Description
Query the textbook knowledge base using RAG (Retrieval Augmented Generation). Returns book-only answers with citations and confidence scores.

## Request

### Headers
- `Content-Type: application/json`
- `Authorization: Bearer {token}` (optional for anonymous users)

### Body
```json
{
  "query": "string (required) - The question or query to search for in the textbook",
  "selected_text": "string (optional) - Text selected by the user for context",
  "module_filter": "string (optional) - Filter results to specific module (e.g., 'ROS2', 'Isaac')",
  "include_citations": "boolean (optional, default: true) - Whether to include citations in response",
  "max_results": "integer (optional, default: 5) - Maximum number of results to return"
}
```

## Response

### Success (200 OK)
```json
{
  "id": "string - Unique identifier for this response",
  "query": "string - The original query",
  "answer": "string - The generated answer based on textbook content",
  "citations": [
    {
      "source": "string - Module/Chapter/Section reference",
      "text": "string - Relevant text snippet",
      "confidence": "float - Confidence score for this citation (0.0-1.0)",
      "page_reference": "string - Page or section reference"
    }
  ],
  "confidence_score": "float - Overall confidence score (0.0-1.0)",
  "response_time_ms": "integer - Time taken to generate response in milliseconds",
  "sources_used": "array - List of content IDs used to generate the answer",
  "selected_text_answer": "boolean - Whether the answer was based on selected text"
}
```

### Error Responses

**400 Bad Request**
```json
{
  "error": "string - Error message",
  "code": "string - Error code",
  "details": "object - Additional error details"
}
```

**429 Rate Limited**
```json
{
  "error": "Rate limit exceeded",
  "retry_after": "integer - Seconds to wait before retrying"
}
```

## Validation Rules
- Query must be between 5 and 1000 characters
- Response time must be under 800ms for 95% of requests
- All answers must be based on book-only content (no external sources)
- Citations must follow APA format
- Confidence score must be between 0.0 and 1.0

## Examples

### Request
```json
{
  "query": "What are the key components of ROS2 architecture?",
  "include_citations": true
}
```

### Response
```json
{
  "id": "rag_abc123",
  "query": "What are the key components of ROS2 architecture?",
  "answer": "The key components of ROS2 architecture include Nodes, which are individual processes that perform computation; Topics, which are named buses over which nodes exchange messages; Services, which provide a request/reply interaction model; and Actions, which are used for long-running tasks with feedback.",
  "citations": [
    {
      "source": "ROS2 Module, Chapter 1, Section 2",
      "text": "ROS2 architecture consists of several key components that work together to enable distributed computing...",
      "confidence": 0.92,
      "page_reference": "ros2/chapter-1#architecture-components"
    }
  ],
  "confidence_score": 0.87,
  "response_time_ms": 420,
  "sources_used": ["content_ros2_arch_001", "content_ros2_intro_002"],
  "selected_text_answer": false
}
```