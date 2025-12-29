# Quickstart: Agentic RAG Backend

## Overview
This guide helps you get started with the agent-based RAG backend system that allows users to ask questions about the humanoid robotics textbook content and receive answers grounded in the book content with source references.

## Prerequisites

Before starting, ensure you have:

- Python 3.10 or higher
- Access to OpenAI API (with proper credentials)
- Access to Cohere API (with proper credentials)
- Access to Qdrant Cloud (with proper credentials)
- UV package manager (if using UV-managed project)

## Environment Setup

### 1. Clone or Navigate to the Project
```bash
cd backend
```

### 2. Install Dependencies
```bash
uv pip install -r requirements.txt
# OR if using pip
pip install -r requirements.txt
```

### 3. Environment Configuration
Create a `.env` file in the backend directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name
```

## Running the Service

### 1. Start the FastAPI Server
```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

The service will be available at `http://localhost:8000`

## Basic Usage

### 1. Stateless Query
Send a single query without maintaining conversation context:

```bash
curl -X POST "http://localhost:8000/api/agent/query" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "query_text": "What are the core components of ROS 2?"
  }'
```

### 2. Session-Based Interaction
Create a session and maintain conversation context:

```bash
# Create a session
curl -X POST "http://localhost:8000/api/agent/session" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "config": {
      "temperature": 0.7,
      "grounding_strictness": 0.5,
      "retrieval_top_k": 5,
      "enable_citations": true
    }
  }'
```

This returns a `session_id` that you'll use for subsequent requests.

```bash
# Use the session for queries
curl -X POST "http://localhost:8000/api/agent/session/{session_id}/query" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "query_text": "Can you explain how ROS 2 nodes communicate?"
  }'
```

## Configuration Options

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| temperature | float | 0.7 | Controls randomness in agent responses (0.0-1.0) |
| grounding_strictness | float | 0.5 | Threshold for grounding responses (0.0-1.0) |
| retrieval_top_k | int | 5 | Number of results to retrieve from Qdrant (1-20) |
| similarity_threshold | float | 0.3 | Minimum similarity for retrieved results (0.0-1.0) |
| response_format | string | "standard" | Format of response ('standard', 'detailed', 'concise') |
| fallback_enabled | bool | true | Enable fallback responses when retrieval fails |
| enable_citations | bool | true | Include source citations in responses |

### Updating Session Configuration
```bash
curl -X PUT "http://localhost:8000/api/agent/session/{session_id}/config" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "config": {
      "temperature": 0.3,
      "enable_citations": true
    }
  }'
```

## Example Response
A typical response will look like:

```json
{
  "response_id": "resp_abc123",
  "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms...",
  "sources": [
    {
      "source_url": "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/docs/module-1-ros2/chapter-1-introduction",
      "page_title": "Chapter 1: Introduction to ROS 2 - The Robotic Nervous System",
      "section_title": "Introduction to ROS 2",
      "chunk_index": 0,
      "content_preview": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
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
4. **Authentication errors**: Ensure your Authorization header is properly formatted

### Checking Service Health
```bash
curl -X GET "http://localhost:8000/health"
```

## Next Steps

1. Integrate with your frontend application
2. Implement proper error handling for your use case
3. Monitor usage and costs for the OpenAI API
4. Fine-tune configuration parameters based on user feedback