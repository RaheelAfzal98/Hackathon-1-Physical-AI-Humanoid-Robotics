# API Contracts for Physical AI & Humanoid Robotics Textbook RAG System

## Overview
This document defines the API contracts for the Retrieval Augmented Generation (RAG) system that powers the textbook's intelligent chatbot, allowing students to ask questions about the content and receive contextually relevant answers grounded in the textbook material.

## Textbook Content API

### GET /api/textbook/modules
List all modules in the textbook

**Response:**
```json
{
  "modules": [
    {
      "id": "mod001",
      "title": "ROS 2 Nervous System",
      "description": "Learn how ROS 2 serves as the communication nervous system of robots",
      "order": 1,
      "chapters": [
        {
          "id": "ch001",
          "title": "Introduction to ROS 2",
          "order": 1
        }
      ]
    }
  ]
}
```

### GET /api/textbook/modules/{module_id}/chapters
List all chapters in a specific module

**Response:**
```json
{
  "module_id": "mod001",
  "chapters": [
    {
      "id": "ch001",
      "title": "Introduction to ROS 2",
      "learning_objectives": ["Understand ROS 2 architecture fundamentals", "Create basic ROS 2 nodes"],
      "prerequisites": []
    }
  ]
}
```

### GET /api/textbook/chapters/{chapter_id}
Get detailed information about a specific chapter

**Response:**
```json
{
  "id": "ch001",
  "module_id": "mod001",
  "title": "Introduction to ROS 2",
  "content": "Markdown content of the chapter...",
  "learning_objectives": ["Understand ROS 2 architecture fundamentals", "Create basic ROS 2 nodes"],
  "prerequisites": [],
  "code_examples": [
    {
      "id": "ce001",
      "title": "Simple ROS 2 Publisher Node",
      "language": "python"
    }
  ],
  "technical_concepts": [
    {
      "id": "tc001",
      "name": "ROS 2 Node"
    }
  ]
}
```

## RAG Query API

### POST /api/rag/query
Submit a question to the RAG system and receive a grounded response

**Request:**
```
Content-Type: application/json
Body:
{
  "query": "What is a ROS 2 node?",
  "context_length": 2048,
  "max_sources": 5,
  "user_id": "optional_user_identifier"
}
```

**Response:**
```json
{
  "query_id": "rq001",
  "query": "What is a ROS 2 node?",
  "response": "A node is an executable that uses ROS 2 client library to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system, and they can publish messages to topics, provide services, or execute actions.",
  "sources": [
    {
      "chapter_id": "ch001",
      "chapter_title": "Introduction to ROS 2",
      "module_id": "mod001",
      "module_title": "ROS 2 Nervous System",
      "text": "A node is an executable that uses ROS 2 client library to communicate with other nodes...",
      "similarity_score": 0.92
    }
  ],
  "groundedness_score": 0.95,
  "query_time_ms": 450
}
```

**Status Codes:**
- 200: Success
- 400: Invalid request (malformed query, invalid parameters)
- 429: Rate limited
- 500: Internal server error (RAG system unavailable)

### POST /api/rag/validate-response
Validate if a response is properly grounded in the textbook content

**Request:**
```
Content-Type: application/json
Body:
{
  "response_text": "A node is an executable that uses ROS 2 client library to communicate with other nodes...",
  "sources": ["rd001", "rd002"],
  "query": "What is a ROS 2 node?"
}
```

**Response:**
```json
{
  "is_validated": true,
  "groundedness_score": 0.95,
  "validation_details": {
    "factual_accuracy": 0.98,
    "source_coverage": 0.89,
    "ungrounded_segments": [],
    "confidence": 0.95
  }
}
```

## RAG Management API

### POST /api/rag/index-content
Index textbook content for RAG retrieval

**Request:**
```
Content-Type: application/json
Body:
{
  "chapter_id": "ch001",
  "content": "Full chapter content to be indexed...",
  "chunk_size": 512,
  "chunk_overlap": 64
}
```

**Response:**
```json
{
  "chapter_id": "ch001",
  "chunks_indexed": 24,
  "index_time_ms": 1200,
  "status": "completed"
}
```

**Status Codes:**
- 200: Indexing completed successfully
- 400: Invalid content or parameters
- 500: Indexing failed

### DELETE /api/rag/remove-content/{chapter_id}
Remove indexed content for a specific chapter

**Response:**
```json
{
  "chapter_id": "ch001",
  "chunks_removed": 24,
  "status": "completed"
}
```

## Search API

### POST /api/search
Semantic search across textbook content

**Request:**
```
Content-Type: application/json
Body:
{
  "query": "How to create a publisher in ROS 2?",
  "top_k": 5,
  "filters": {
    "module_ids": ["mod001"],
    "chapter_ids": []
  }
}
```

**Response:**
```json
{
  "query": "How to create a publisher in ROS 2?",
  "results": [
    {
      "id": "rd001",
      "chapter_id": "ch001",
      "chapter_title": "Introduction to ROS 2",
      "module_id": "mod001",
      "module_title": "ROS 2 Nervous System",
      "content": "To create a publisher in ROS 2, you need to initialize the publisher within a node using create_publisher...",
      "similarity_score": 0.89
    }
  ]
}
```