# Quickstart Guide: RAG Retrieval Validation

## Overview

This guide provides a quick introduction to setting up and using the RAG retrieval validation system. By following these steps, you'll be able to connect to Qdrant Cloud, perform semantic searches against your embedded book content, and validate the retrieval pipeline.

## Prerequisites

- Python 3.11+
- Access to Cohere API (for generating embeddings)
- Access to Qdrant Cloud (with a collection containing embedded book content)
- UV package manager (recommended) or pip

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-directory>
```

2. Install dependencies using UV (recommended) or pip:
```bash
# Using UV
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -r backend/requirements.txt

# Or using pip
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r backend/requirements.txt
```

3. Set up environment variables:
```bash
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_URL="your-qdrant-cloud-url"
export QDRANT_API_KEY="your-qdrant-api-key"
export QDRANT_COLLECTION_NAME="your-collection-name"
```

## Basic Usage

### 1. Starting the API Server

```bash
cd backend
uv run python -m src.main
```

Or using the startup script:
```bash
./scripts/start-server.sh
```

The server will start on `http://localhost:8000`.

### 2. Performing a Retrieval Query

Using curl:

```bash
curl -X POST http://localhost:8000/rag/retrieve \
  -H "Content-Type: application/json" \
  -d '{
    "text": "How do humanoid robots maintain balance?",
    "top_k": 5,
    "similarity_threshold": 0.7
  }'
```

Using Python:

```python
import requests

response = requests.post(
    "http://localhost:8000/rag/retrieve",
    json={
        "text": "How do humanoid robots maintain balance?",
        "top_k": 5,
        "similarity_threshold": 0.7
    }
)

results = response.json()
print(f"Found {len(results['results'])} relevant chunks")
for chunk in results['results']:
    print(f"Score: {chunk['similarity_score']:.2f}")
    print(f"Content: {chunk['chunk']['content'][:200]}...")
    print(f"Source: {chunk['chunk']['metadata']['source_url']}")
    print("---")
```

### 3. Validating the Retrieval Pipeline

```bash
curl -X GET http://localhost:8000/rag/validate-pipeline
```

## Configuration

The system can be configured through environment variables:

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: URL of your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_COLLECTION_NAME`: Name of the collection containing embedded content
- `QDRANT_PORT`: Port for Qdrant connection (default: 6333)
- `EMBEDDING_MODEL`: Cohere embedding model to use (default: "embed-multilingual-v2.0")

## Example Response

A typical retrieval response looks like:

```json
{
  "query_text": "How do humanoid robots maintain balance?",
  "results": [
    {
      "chunk": {
        "id": "ch04-012",
        "content": "Humanoid robots maintain balance using...",
        "metadata": {
          "source_url": "/chapters/ch04-balancing-mechanisms.md",
          "page_title": "Balancing Mechanisms",
          "section": "Control Systems",
          "chunk_index": 12,
          "document_id": "ch04-balancing-mechanisms"
        },
        "created_at": "2025-12-18T10:00:00Z"
      },
      "similarity_score": 0.87,
      "rank": 0
    }
  ],
  "total_chunks_processed": 150,
  "search_time_ms": 35.2,
  "retrieval_parameters": {
    "top_k": 5,
    "similarity_threshold": 0.7,
    "collection_name": "book_embeddings"
  }
}
```

## Testing

Run the test suite to validate functionality:

```bash
# Unit tests
python -m pytest tests/unit/

# Integration tests
python -m pytest tests/integration/

# Contract tests
python -m pytest tests/contract/

# All tests
python -m pytest
```

### Running the Validation Script

The project includes a comprehensive validation script to test the entire pipeline:

```bash
cd backend
python scripts/validate_rag.py
# Or to test a specific endpoint:
python scripts/validate_rag.py http://your-server-address:8000
```

This script will:
- Validate environment configuration
- Test API connectivity
- Perform sample queries
- Verify source traceability
- Report comprehensive validation results

### Ingesting Content

Before you can retrieve content, you need to populate your Qdrant database with embeddings. You can do this in two ways:

**Option 1: Using the command-line script**

```bash
cd backend
python scripts/ingest_content.py --urls https://your-book-site.com/docs https://your-book-site.com/guides
```

Or with custom chunking parameters:
```bash
python scripts/ingest_content.py --urls https://your-book-site.com/docs --chunk-size 1024 --overlap 100
```

**Option 2: Using the API endpoint**

```bash
curl -X POST http://localhost:8000/rag/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "urls": ["https://your-book-site.com/docs", "https://your-book-site.com/guides"]
  }'
```

The ingestion process will:
- Crawl the specified URLs
- Extract clean text content
- Chunk the content into manageable pieces
- Generate embeddings using Cohere
- Store the embeddings in Qdrant with metadata

## Development

### Project Structure

```
backend/
├── src/
│   ├── rag/
│   │   ├── models.py          # Pydantic models for requests/responses
│   │   ├── client.py          # Qdrant client wrapper
│   │   ├── embedding.py       # Cohere embedding utilities
│   │   ├── retrieval.py       # Main retrieval logic
│   │   └── validation_service.py  # Validation functionality
│   ├── config/
│   │   └── settings.py        # Configuration settings
│   └── api/
│       └── routes/
│           └── rag.py         # RAG API endpoints
├── tests/
│   ├── unit/
│   │   └── test_retrieval.py  # Unit tests for retrieval
│   ├── integration/
│   │   └── test_api.py        # Integration tests for API
│   └── contract/
│       └── test_contracts.py  # Contract tests
└── scripts/
    └── validate_rag.py        # End-to-end validation script
```

### Using the Validation Service

You can also use the validation service programmatically:

```python
from src.rag.validation_service import ValidationService

validator = ValidationService()

# Validate pipeline connectivity
connectivity_results = validator.validate_pipeline_connectivity()
print(connectivity_results)

# Validate source traceability for content chunks
from src.rag.models import ContentChunk

sample_chunk = ContentChunk(
    id="test-123",
    content="Sample content for validation",
    metadata={
        "source_url": "/test/source.md",
        "page_title": "Test Page",
        "document_id": "doc1",
        "chunk_index": 1
    }
)

traceability_results = validator.validate_source_traceability([sample_chunk])
print(traceability_results)
```

## Troubleshooting

Common issues and solutions:

1. **Connection errors to Qdrant Cloud**: Verify your `QDRANT_URL` and `QDRANT_API_KEY` environment variables
2. **Invalid embeddings**: Check that your `COHERE_API_KEY` is valid and has sufficient quota
3. **Empty results**: Lower the `similarity_threshold` or verify that your collection contains properly embedded content
4. **Slow performance**: Consider optimizing your Qdrant instance settings or indexing strategies
5. **Validation failures**: Run the validation script to identify specific issues: `python scripts/validate_rag.py`

## Next Steps

- Explore the full API documentation at `/docs` when the server is running
- Review the data models in `specs/005-rag-retrieval-validation/data-model.md`
- Examine the validation scripts in the `scripts/` directory
- Run the comprehensive validation script to ensure everything is working