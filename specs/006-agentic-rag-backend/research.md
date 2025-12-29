# Research: Agentic RAG Backend

## Executive Summary

This research addresses the implementation of an agent-based RAG system using OpenAI Agents SDK with FastAPI, integrating with the existing Qdrant-based retrieval pipeline. The system will enable users to ask questions about the humanoid robotics textbook content and receive answers grounded in the book content with source references.

## Key Decisions

### 1. Agent Framework Selection
**Decision**: Use OpenAI Assistants API as the agent framework
**Rationale**: 
- OpenAI Assistants API provides a managed agent system with built-in memory, conversation history, and tool integration
- Well-documented and maintained by OpenAI
- Integrates well with existing Python backend via official SDK
- Supports tool-based function calling which is perfect for the retrieval requirement

**Alternatives considered**:
- Custom agent implementations using LangChain or similar
- Open-source alternatives like AutoGen
- Native OpenAI function calling without the Assistants API

### 2. Retrieval Tool Integration
**Decision**: Implement the existing Qdrant retrieval as a custom tool for the OpenAI Assistant
**Rationale**:
- The spec requires integration with existing retrieval pipeline
- OpenAI Assistants API supports custom tools that can be called during agent execution
- Ensures responses remain grounded in the book content
- Matches the requirement for tool-based retrieval

**Alternatives considered**:
- Passing retrieved content directly in the initial query
- Creating a separate service layer between agent and retrieval
- Using retrieval-augmented generation at the model level

### 3. Session Management Approach
**Decision**: Use OpenAI Assistant threads for conversation context and create a custom session layer for application-specific state
**Rationale**:
- OpenAI Assistants API includes Thread objects for conversation context
- Need additional session layer for application-specific state (config, user preferences)
- Threads handle conversation history automatically
- Can map application sessions to Assistant threads

**Alternatives considered**:
- Pure stateless interactions
- Implementing full conversation context from scratch
- Different session management libraries

## Technical Implementation Details

### Agent Architecture
```
User Query → FastAPI Endpoint → Assistant Thread → Retrieval Tool → Grounded Response
```

The agent will be structured as follows:
1. FastAPI endpoints receive user queries
2. Create or retrieve existing assistant thread
3. Run assistant with query and access to retrieval tool
4. Assistant decides whether to use retrieval tool based on the query
5. Tool retrieves relevant content from Qdrant
6. Assistant synthesizes response using retrieved content
7. Response includes source references

### Retrieval Tool Functionality
The custom tool will implement the following:
- Accept a search query from the assistant
- Call existing Qdrant retrieval pipeline
- Return formatted results that the assistant can understand
- Include metadata and source references

### Session Handling
For stateless interactions:
- Create temporary thread for each request
- Return response immediately

For session-based interactions:
- Create persistent session with associated thread ID
- Maintain context across multiple requests
- Allow for proper conversation history management

### Configuration System
The configuration system will support:
- Agent behavior parameters (temperature, grounding strictness)
- Retrieval parameters (number of results, similarity threshold)
- Response formatting options
- Fallback response settings

## Integration with Existing Components

### Backend Integration
The new agent system will integrate with:
- Existing Qdrant retrieval pipeline in `src/rag/retrieval.py`
- Current FastAPI application structure
- Configuration management system in `src/config/settings.py`
- Cohere embedding service

### API Design Considerations
The API endpoints will follow the patterns:
- `/api/agent/query` - Stateless query endpoint
- `/api/agent/session/create` - Create new session
- `/api/agent/session/{session_id}/query` - Session-based query
- `/api/agent/session/{session_id}/config` - Configure session parameters

## Risk Assessment

### Potential Challenges
1. **Rate Limits**: OpenAI API has rate limits that may impact concurrent users
2. **Cost Management**: Assistant API usage incurs costs that need monitoring
3. **Response Consistency**: Ensuring agents consistently ground responses in retrieved content
4. **Error Handling**: Managing failures in both agent service and retrieval service

### Mitigation Strategies
1. Implement retry logic and circuit breakers
2. Add cost monitoring and usage limits
3. Use grounding validation to verify response sources
4. Create comprehensive fallback strategies

## Performance Considerations

Based on the success criteria, the system should:
- Return responses within acceptable timeframes for user experience
- Handle multiple concurrent user sessions
- Maintain quality of responses under load
- Efficiently utilize both OpenAI API and Qdrant resources

## Compliance with Requirements

All functional requirements from the specification will be addressed:
- ✅ Agent instantiated using OpenAI Agents SDK
- ✅ Retrieval exposed as a tool for the agent
- ✅ Responses grounded in book content
- ✅ Source references included in responses
- ✅ API endpoints for agent interactions
- ✅ Support for stateless and session-based interactions
- ✅ Graceful handling of retrieval failures
- ✅ Configurable agent behavior