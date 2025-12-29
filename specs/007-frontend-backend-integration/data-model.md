# Data Model: Frontend-Backend RAG Integration

## Entity Relationships

### QueryRequest
**Description**: User input containing the question and optional session context
**Fields**:
- `query_text`: String - The actual question or query text (1-2000 characters)
- `session_id`: String? - Optional session ID for stateful interactions
- `selected_text`: String? - Text selected by the user to provide additional context (optional)
- `user_context`: Dict? - Additional context about the user or situation (max 5KB)
- `response_options`: Dict? - Specific response options (overrides session config)

**Relationships**:
- Belongs to AgentSession (optional)

### AgentResponse
**Description**: Structured response containing the answer text, source references, confidence indicators, and metadata
**Fields**:
- `response_id`: String - Unique identifier for the response
- `content`: String - The main response text from the agent (max 4000 characters)
- `sources`: List<SourceReference> - List of sources used in the response
- `confidence`: Float - Confidence level of the response (0.0-1.0)
- `tool_calls`: List<ToolCall> - List of tools called during response generation
- `created_at`: DateTime - Timestamp when response was generated
- `query_time_ms`: Integer - Time taken to generate the response in milliseconds

**Relationships**:
- Belongs to AgentSession (optional)

### SourceReference
**Description**: Reference to original content that grounds an agent response
**Fields**:
- `source_url`: String - URL where the content can be found
- `page_title`: String - Title of the page or document
- `section_title`: String - Specific section or heading
- `chunk_index`: Integer - Index of the content chunk
- `content_preview`: String - Short preview of the referenced content (max 500 chars)
- `similarity_score`: Float - How similar this content is to the query (0.0-1.0)
- `relevance_score`: Float - How relevant this content is to the response (0.0-1.0)

### ToolCall
**Description**: A function call made by the agent during response generation
**Fields**:
- `tool_name`: String - Name of the tool that was called
- `arguments`: Dict - Arguments passed to the tool
- `result`: String - Result returned by the tool
- `timestamp`: DateTime - When the tool was called

### AgentSession
**Description**: Represents a conversation context including conversation history, user preferences, and temporary state
**Fields**:
- `session_id`: String - Unique identifier for the session
- `created_at`: DateTime - Timestamp when session was created
- `last_interaction`: DateTime - Timestamp of last interaction
- `config`: AgentConfiguration - Session-specific configuration

**Relationships**:
- One-to-Many with AgentInteraction (one session can have multiple interactions)

### AgentConfiguration
**Description**: Parameters that control agent behavior including response style, grounding strictness, and tool usage preferences
**Fields**:
- `temperature`: Float - Controls randomness in agent responses (0.0-1.0)
- `grounding_strictness`: Float - Threshold for how strictly to ground responses (0.0-1.0)
- `retrieval_top_k`: Integer - Number of results to retrieve from the vector database (1-20)
- `similarity_threshold`: Float - Minimum similarity score for retrieved results (0.0-1.0)
- `response_format`: String - Desired response format ('standard', 'detailed', 'concise')
- `fallback_enabled`: Boolean - Whether to use fallback responses when retrieval fails
- `enable_citations`: Boolean - Whether to include citations in responses

## State Transitions

### AgentSession States
```
PENDING → ACTIVE → INACTIVE → ARCHIVED
```
- PENDING: Session created but no interaction yet
- ACTIVE: Session has received at least one query in the last 30 minutes
- INACTIVE: Session has not been used for >30 minutes but within retention period
- ARCHIVED: Session has exceeded retention period and is archived

### Response Processing States
```
RECEIVED → PROCESSING → GROUNDED → VALIDATED → DELIVERED
```
- RECEIVED: Query has been received by the system
- PROCESSING: Agent is processing the query and may call tools
- GROUNDED: Response is being grounded in retrieved content
- VALIDATED: Response has been validated for quality and grounding
- DELIVERED: Response has been delivered to the user

## Validation Rules

### AgentConfiguration Validation
- Temperature must be between 0.0 and 1.0
- Grounding_strictness must be between 0.0 and 1.0
- Retrieval_top_k must be between 1 and 20
- Similarity_threshold must be between 0.0 and 1.0

### QueryRequest Validation
- query_text must be between 1 and 2000 characters
- session_id (if provided) must exist and be active
- user_context (if provided) must not exceed 5KB

### AgentResponse Validation
- content must not exceed 4000 characters
- sources list must not exceed 10 items
- confidence score must be between 0.0 and 1.0
- All sources must have valid URLs

### SourceReference Validation
- source_url must be a valid URL
- similarity_score and relevance_score must be between 0.0 and 1.0
- content_preview must not exceed 500 characters