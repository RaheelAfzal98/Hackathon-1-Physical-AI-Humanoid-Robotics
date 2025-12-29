# Data Model: RAG Retrieval Validation

## Entities

### Query
**Description:** Natural-language input from users that is converted to embeddings for similarity search
**Fields:**
- `text`: str - The natural language query text
- `top_k`: int - Number of results to retrieve (default: 5, range: 1-20)
- `similarity_threshold`: float - Minimum similarity score for inclusion (default: 0.5, range: 0.0-1.0)
- `filters`: dict - Optional filters for metadata-based query refinement

### Content Chunk
**Description:** Individual segments of book content stored in the vector database with embeddings and metadata
**Fields:**
- `id`: str - Unique identifier for the chunk
- `content`: str - The actual text content of the chunk
- `embedding`: List[float] - Vector representation of the content (1024 dimensions for Cohere)
- `metadata`: Dict[str, Any] - Additional information about the chunk
  - `source_url`: str - Original URL or file path where the content originated
  - `page_title`: str - Title of the page/chapter containing the content
  - `section`: str - Section or subsection header
  - `chunk_index`: int - Sequential position of this chunk in the original content
  - `document_id`: str - Identifier for the parent document
- `created_at`: datetime - Timestamp when the chunk was created

### Retrieval Result
**Description:** System response containing relevant content chunks with metadata and relevance scores
**Fields:**
- `query_text`: str - The original query text submitted by the user
- `results`: List[RankedChunk] - Ranked list of relevant content chunks
- `total_chunks_processed`: int - Total number of chunks considered during retrieval
- `search_time_ms`: float - Time taken to perform the retrieval operation
- `retrieval_parameters`: RetrievalParameters - Parameters used for this specific retrieval

### RankedChunk
**Description:** A content chunk with its relevance score and ranking position
**Fields:**
- `chunk`: ContentChunk - The original content chunk
- `similarity_score`: float - Cosine similarity score between query and chunk (0.0-1.0)
- `rank`: int - Position in the ranked results (0-indexed)

### RetrievalParameters
**Description:** Configuration parameters for the retrieval operation
**Fields:**
- `top_k`: int - Maximum number of results to return
- `similarity_threshold`: float - Minimum similarity score for inclusion
- `collection_name`: str - Name of the vector database collection to search
- `filters`: Optional[dict] - Filters to apply during search

### ValidationError
**Description:** Error response when validation fails
**Fields:**
- `error_code`: str - Standardized error code
- `message`: str - Human-readable error description
- `details`: Optional[Dict[str, Any]] - Additional context about the error
- `timestamp`: datetime - When the error occurred

## Relationships

```
Query 1 -----> * RankedChunk
RankedChunk 1 -----> 1 ContentChunk
RetrievalResult 1 -----> * RankedChunk
RetrievalResult 1 -----> 1 RetrievalParameters
```

## Validation Rules

### Query Validation
- `text` must be between 1-1000 characters
- `top_k` must be between 1-20
- `similarity_threshold` must be between 0.0-1.0

### ContentChunk Validation
- `id` must be unique within the collection
- `content` must not exceed 10,000 characters
- `embedding` must have the correct dimensionality (matching model)
- `metadata.source_url` must be a valid URL or file path
- `chunk_index` must be non-negative

### RankedChunk Validation
- `similarity_score` must be between 0.0-1.0
- `rank` must be non-negative and sequential

## State Transitions

### Retrieval Process
1. **Pending**: Query received and validated
2. **Processing**: Query transformed to embedding, vector search initiated
3. **Scoring**: Retrieved chunks scored for relevance
4. **Filtered**: Results filtered by similarity threshold
5. **Ranked**: Final results ranked by relevance
6. **Complete**: Response assembled with metadata

## JSON Schema Example

### Query Request
```json
{
  "text": "How do humanoid robots achieve bipedal locomotion?",
  "top_k": 5,
  "similarity_threshold": 0.7,
  "filters": {
    "document_id": "ch04-humanoid-locomotion"
  }
}
```

### Retrieval Response
```json
{
  "query_text": "How do humanoid robots achieve bipedal locomotion?",
  "results": [
    {
      "chunk": {
        "id": "ch04-001",
        "content": "Humanoid robots achieve bipedal locomotion through a combination of advanced control algorithms including ZMP (Zero Moment Point) control and whole-body motion controllers...",
        "metadata": {
          "source_url": "/chapters/ch04-humanoid-locomotion.md",
          "page_title": "Humanoid Locomotion",
          "section": "Control Systems",
          "chunk_index": 1,
          "document_id": "ch04-humanoid-locomotion"
        },
        "created_at": "2025-12-18T10:00:00Z"
      },
      "similarity_score": 0.85,
      "rank": 0
    }
  ],
  "total_chunks_processed": 125,
  "search_time_ms": 45.2,
  "retrieval_parameters": {
    "top_k": 5,
    "similarity_threshold": 0.7,
    "collection_name": "book_embeddings"
  }
}
```