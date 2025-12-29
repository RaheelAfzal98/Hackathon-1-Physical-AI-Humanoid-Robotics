# Quickstart Guide: Frontend-Backend RAG Integration

## Overview
This guide helps you get started with the agent-based RAG backend system that enables frontend applications to query the humanoid robotics textbook content and receive grounded answers with source references.

## Prerequisites
- Python 3.10+
- Access to OpenAI API (for agent functionality)
- Access to Cohere API (for embeddings)
- Access to Qdrant Cloud (for vector storage)
- Docusaurus site deployed and accessible

## Setup Instructions

### 1. Clone the repository
```bash
git clone <repository-url>
cd backend
```

### 2. Install dependencies
```bash
pip install -r requirements.txt
```

### 3. Configure environment variables
Create a `.env` file in the backend root with the following:
```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name
```

### 4. Start the backend service
```bash
# Navigate to backend directory
cd backend

# Start the FastAPI server
uvicorn src.rag.api.main:app --reload --port 8000
```

The service will be available at `http://localhost:8000`

## Basic Usage

### 1. Stateless Query
Send a single query without maintaining session context:
```bash
curl -X POST "http://localhost:8000/agent/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What is ROS 2 and how does it apply to humanoid robots?",
    "response_options": {
      "temperature": 0.7,
      "enable_citations": true
    }
  }'
```

### 2. Session-Based Interaction
Create a session and maintain conversation context:
```bash
# Create a session
curl -X POST "http://localhost:8000/agent/session" \
  -H "Content-Type: application/json" \
  -d '{
    "config": {
      "temperature": 0.5,
      "grounding_strictness": 0.7,
      "retrieval_top_k": 5
    }
  }'
```

This returns a `session_id` that you'll use for subsequent requests.

```bash
# Make a query within the session context
curl -X POST "http://localhost:8000/agent/session/{session_id}/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "Explain the key components of a ROS 2 system",
    "selected_text": "The Robot Operating System (ROS) is a flexible framework for writing robot software"
  }'
```

## Configuration Options

### Available Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| temperature | float | 0.7 | Controls randomness in agent responses (0.0-1.0) |
| grounding_strictness | float | 0.5 | Threshold for grounding responses (0.0-1.0) |
| retrieval_top_k | int | 5 | Number of results to retrieve (1-20) |
| similarity_threshold | float | 0.3 | Minimum similarity for retrieved results (0.0-1.0) |
| response_format | string | "standard" | Format of response ('standard', 'detailed', 'concise') |
| fallback_enabled | bool | true | Enable fallback responses when retrieval fails |
| enable_citations | bool | true | Include source citations in responses |

### Updating Session Configuration
```bash
curl -X PUT "http://localhost:8000/agent/session/{session_id}/config" \
  -H "Content-Type: application/json" \
  -d '{
    "config": {
      "temperature": 0.3,
      "enable_citations": false
    }
  }'
```

## Example Response
A typical response will look like:
```json
{
  "response_id": "resp_abc123",
  "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
  "sources": [
    {
      "source_url": "https://docs.ros.org/en/humble/The-ROS2-Project/Concepts.html",
      "page_title": "ROS 2 Concepts",
      "section_title": "Introduction to ROS 2",
      "chunk_index": 0,
      "content_preview": "ROS 2 is a flexible framework for writing robot software...",
      "similarity_score": 0.85,
      "relevance_score": 0.9
    }
  ],
  "confidence": 0.88,
  "tool_calls": ["retrieval_tool"],
  "created_at": "2025-12-19T10:30:00Z",
  "query_time_ms": 1245
}
```

## Troubleshooting

### Common Issues
1. **API Keys not working**: Verify your environment variables are properly set
2. **Slow responses**: Check your API rate limits and potentially adjust configuration
3. **No results for queries**: Verify your Qdrant collection has data and the retrieval service is working
4. **Session not found errors**: Ensure you're using a valid session_id from a successful session creation

### Checking Service Health
```bash
curl -X GET "http://localhost:8000/health"
```

## Advanced Usage

### Contextual Querying
To submit a query with selected text context:
```bash
curl -X POST "http://localhost:8000/agent/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "Explain this concept in more detail",
    "selected_text": "Embodied AI refers to artificial intelligence that interacts with the physical world through robotic systems. This field combines machine learning, robotics, and perception to create intelligent agents that can operate in real environments.",
    "user_context": {
      "user_role": "student",
      "knowledge_level": "intermediate"
    }
  }'
```

This allows the agent to focus specifically on the provided text when generating a response.