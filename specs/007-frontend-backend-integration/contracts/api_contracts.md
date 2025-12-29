# API Contracts: Frontend-Backend RAG Integration

## Overview
This document defines the API contracts for the agent-based RAG backend system. The system provides endpoints for frontend applications to query the humanoid robotics textbook content and receive grounded answers with source references.

## Base URL
`/api/agent`

## Authentication
All endpoints require authentication using API key in the header:
`Authorization: Bearer {API_KEY}`

## Common Data Types

### SourceReference
```json
{
  "source_url": "string, URL of the original document",
  "page_title": "string, title of the page or document",
  "section_title": "string, specific section heading",
  "chunk_index": "integer, index of the content chunk",
  "content_preview": "string, short preview of the referenced content (max 500 chars)",
  "similarity_score": "number, how similar this content is to the query (0.0-1.0)",
  "relevance_score": "number, how relevant this content is to the response (0.0-1.0)"
}
```

### AgentConfiguration
```json
{
  "temperature": "number, controls randomness in agent responses (0.0-1.0)",
  "grounding_strictness": "number, threshold for grounding responses (0.0-1.0)",
  "retrieval_top_k": "integer, number of results to retrieve from Qdrant (1-20)",
  "similarity_threshold": "number, minimum similarity score for retrieved results (0.0-1.0)",
  "response_format": "string, desired response format ('standard', 'detailed', 'concise')",
  "fallback_enabled": "boolean, whether to use fallback responses when retrieval fails",
  "enable_citations": "boolean, whether to include citations in responses"
}
```

## Endpoints

### 1. Stateless Query Endpoint

**POST** `/api/agent/query`

Query the agent without maintaining session context.

#### Request
```json
{
  "query_text": "string, the question to ask the agent",
  "response_options": {
    "temperature": "number, optional override for temperature",
    "grounding_strictness": "number, optional override for grounding strictness",
    "retrieval_top_k": "integer, optional override for number of results",
    "enable_citations": "boolean, optional override for citation inclusion"
  }
}
```

#### Response (Success: 200 OK)
```json
{
  "response_id": "string, unique identifier for the response",
  "content": "string, the agent's response to the query",
  "sources": "array of SourceReference, sources used to ground the response",
  "confidence": "number, confidence level of the response (0.0-1.0)",
  "tool_calls": "array of strings, tools called during response generation",
  "created_at": "string, ISO 8601 timestamp when response was generated",
  "query_time_ms": "integer, time taken to generate the response in milliseconds"
}
```

#### Response (Error: 400 Bad Request)
```json
{
  "error": "string, description of the error",
  "code": "string, error code",
  "details": "object, additional error details if applicable"
}
```

#### Response (Error: 500 Internal Server Error)
```json
{
  "error": "string, description of the server error",
  "code": "string, error code"
}
```

### 2. Create Session

**POST** `/api/agent/session`

Create a new agent session for maintaining conversation context.

#### Request
```json
{
  "config": {
    "temperature": "number, controls randomness in agent responses (0.0-1.0)",
    "grounding_strictness": "number, threshold for grounding responses (0.0-1.0)",
    "retrieval_top_k": "integer, number of results to retrieve from Qdrant (1-20)",
    "similarity_threshold": "number, minimum similarity score for retrieved results (0.0-1.0)",
    "response_format": "string, desired response format ('standard', 'detailed', 'concise')",
    "fallback_enabled": "boolean, whether to use fallback responses when retrieval fails",
    "enable_citations": "boolean, whether to include citations in responses"
  },
  "initial_context": "object, optional initial context for the session"
}
```

#### Response (Success: 201 Created)
```json
{
  "session_id": "string, unique identifier for the created session",
  "created_at": "string, ISO 8601 timestamp when session was created",
  "config": "AgentConfiguration, the configuration for this session"
}
```

### 3. Session Query

**POST** `/api/agent/session/{session_id}/query`

Query the agent within a session context, maintaining conversation history.

#### Request
```json
{
  "query_text": "string, the question to ask the agent",
  "selected_text_context": "string, optional text that the user has selected on the page",
  "response_options": {
    "temperature": "number, optional override for temperature",
    "grounding_strictness": "number, optional override for grounding strictness",
    "retrieval_top_k": "integer, optional override for number of results",
    "enable_citations": "boolean, optional override for citation inclusion"
  }
}
```

#### Response (Success: 200 OK)
```json
{
  "response_id": "string, unique identifier for the response",
  "content": "string, the agent's response to the query",
  "sources": "array of SourceReference, sources used to ground the response",
  "confidence": "number, confidence level of the response (0.0-1.0)",
  "tool_calls": "array of strings, tools called during response generation",
  "created_at": "string, ISO 8601 timestamp when response was generated",
  "query_time_ms": "integer, time taken to generate the response in milliseconds"
}
```

### 4. Get Session Configuration

**GET** `/api/agent/session/{session_id}/config`

Retrieve the configuration for a specific session.

#### Response (Success: 200 OK)
```json
{
  "config": "AgentConfiguration, the configuration for this session"
}
```

### 5. Update Session Configuration

**PUT** `/api/agent/session/{session_id}/config`

Update the configuration for a specific session.

#### Request
```json
{
  "config": "AgentConfiguration, the new configuration for this session"
}
```

#### Response (Success: 200 OK)
```json
{
  "config": "AgentConfiguration, the updated configuration for this session"
}
```

### 6. List Sessions

**GET** `/api/agent/sessions`

List active sessions (for monitoring purposes).

#### Response (Success: 200 OK)
```json
{
  "sessions": [
    {
      "session_id": "string, unique identifier for the session",
      "created_at": "string, ISO 8601 timestamp when session was created",
      "last_interaction": "string, ISO 8601 timestamp of last interaction",
      "status": "string, current status of the session (active, inactive, archived)"
    }
  ],
  "total_count": "integer, total number of sessions matching the query"
}
```

### 7. Delete Session

**DELETE** `/api/agent/session/{session_id}`

Delete a session (useful for cleaning up or when user ends conversation).

#### Response (Success: 204 No Content)

## Error Responses

The API uses standard HTTP status codes:
- `200`: Success
- `201`: Created
- `204`: No Content
- `400`: Bad Request (invalid input)
- `401`: Unauthorized
- `404`: Not Found
- `500`: Internal Server Error

## Validation Rules

### Request Validation
- All required fields must be present
- String lengths must not exceed specified limits
- Numeric values must be within specified ranges
- URLs must be properly formatted

### Response Validation
- Responses must include all required fields
- Confidence scores must be between 0.0 and 1.0
- Source references must include valid URLs
- Timestamps must be in ISO 8601 format