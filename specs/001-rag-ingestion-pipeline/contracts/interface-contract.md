# API Contract: RAG Ingestion Pipeline Interface

## Overview
This document defines the API contracts for the RAG ingestion pipeline that extracts book content from deployed website URLs, generates embeddings using Cohere, and stores them in Qdrant Cloud for RAG-based retrieval.

## Service Interface

### Main Ingestion Service (`app.main`)

#### Function: `get_all_urls(base_url)`
- **Purpose**: Discover and return all accessible URLs from a Docusaurus website
- **Input**: 
  - `base_url` (string): The base URL of the Docusaurus site to crawl
- **Output**: `List[str]` - List of all discovered URLs
- **Errors**:
  - NetworkError: When the base URL is inaccessible
  - InvalidUrlError: When the base URL is malformed
- **Notes**: Implements sitemap.xml discovery with fallback to link crawling

#### Function: `extract_text_from_url(url)`
- **Purpose**: Extract clean text content from a single web page
- **Input**: 
  - `url` (string): The URL to extract content from
- **Output**: `Dict` - Dictionary containing:
  - `text` (string): Cleaned text content
  - `title` (string): Page title
  - `hash` (string): Content hash for change detection
- **Errors**:
  - UrlFetchError: When the URL cannot be retrieved
  - ContentExtractionError: When text extraction fails
- **Notes**: Removes navigation, headers, footers, and other non-content elements

#### Function: `chunk_text(text_content, chunk_size=512, chunk_overlap=50)`
- **Purpose**: Split text content into overlapping chunks of specified size
- **Input**:
  - `text_content` (string): Text to be chunked
  - `chunk_size` (int): Maximum size of each chunk (default: 512)
  - `chunk_overlap` (int): Overlap size between consecutive chunks (default: 50)
- **Output**: `List[Dict]` - List of dictionaries containing:
  - `text` (string): Chunked text content
  - `chunk_index` (int): Position index of this chunk
  - `total_chunks` (int): Total number of chunks in the document
- **Errors**:
  - ValidationError: When chunk parameters are invalid
- **Notes**: Maintains semantic coherence across chunk boundaries

#### Function: `embed(text_chunks, model='embed-multilingual-v3.0')`
- **Purpose**: Generate embeddings for text chunks using Cohere API
- **Input**:
  - `text_chunks` (List[string]): List of text chunks to embed
  - `model` (string): Cohere embedding model to use (default: 'embed-multilingual-v3.0')
- **Output**: `List[List[float]]` - List of embedding vectors, each being a list of floats
- **Errors**:
  - CohereApiError: When Cohere API is inaccessible or returns an error
  - RateLimitError: When Cohere API rate limits are exceeded
- **Notes**: Implements exponential backoff and retry logic

#### Function: `create_collection(collection_name, vector_size, distance='Cosine')`
- **Purpose**: Create a Qdrant collection to store embeddings
- **Input**:
  - `collection_name` (string): Name of the collection to create
  - `vector_size` (int): Dimensionality of vectors to be stored
  - `distance` (string): Distance metric for similarity search (default: 'Cosine')
- **Output**: Boolean - True if collection was created or already exists
- **Errors**:
  - QdrantApiError: When Qdrant API is inaccessible or returns an error
- **Notes**: Idempotent operation that doesn't fail if collection already exists

#### Function: `save_chunk_to_qdrant(chunk_data, collection_name, vector)`
- **Purpose**: Save a content chunk with its embedding to Qdrant
- **Input**:
  - `chunk_data` (Dict): Dictionary with chunk information (text, source_url, etc.)
  - `collection_name` (string): Name of collection to store the vector
  - `vector` (List[float]): Embedding vector to store
- **Output**: String - The ID of the stored point in Qdrant
- **Errors**:
  - QdrantApiError: When Qdrant API is inaccessible or returns an error
- **Notes**: Uses SHA256 hash of URL+content+index as the point ID for idempotency

#### Function: `execute_ingestion_pipeline(base_url, config)`
- **Purpose**: Execute the complete ingestion pipeline from URL discovery to vector storage
- **Input**:
  - `base_url` (string): Base URL of Docusaurus site to ingest
  - `config` (PipelineConfig): Configuration object with all processing parameters
- **Output**: Dict with statistics about the ingestion:
  - `urls_processed` (int): Number of URLs successfully processed
  - `chunks_created` (int): Number of content chunks created
  - `vectors_stored` (int): Number of vectors stored in Qdrant
  - `errors` (List[str]): List of any errors that occurred
- **Errors**:
  - ConfigurationError: When configuration parameters are invalid
  - ProcessingError: When the pipeline fails to complete successfully
- **Notes**: Implements the full workflow: get_all_urls -> extract_text_from_url -> chunk_text -> embed -> save_chunk_to_qdrant

## Configuration Schema

### PipelineConfig Object
```json
{
  "chunk_size": {
    "type": "integer",
    "minimum": 1,
    "maximum": 2048,
    "default": 512
  },
  "chunk_overlap": {
    "type": "integer",
    "minimum": 0,
    "maximum": 2047,
    "default": 50
  },
  "embedding_model": {
    "type": "string",
    "default": "embed-multilingual-v3.0",
    "enum": [
      "embed-multilingual-v3.0",
      "embed-english-v3.0"
    ]
  },
  "collection_name": {
    "type": "string"
  },
  "batch_size": {
    "type": "integer",
    "minimum": 1,
    "maximum": 50,
    "default": 10
  },
  "max_retries": {
    "type": "integer",
    "minimum": 0,
    "maximum": 10,
    "default": 3
  },
  "delay_base": {
    "type": "number",
    "minimum": 0.1,
    "maximum": 10.0,
    "default": 1.0
  },
  "timeout": {
    "type": "integer",
    "minimum": 1,
    "maximum": 300,
    "default": 30
  },
  "enable_change_detection": {
    "type": "boolean",
    "default": true
  }
}
```

## Error Definitions

### NetworkError
- **Code**: INGESTION_001
- **Message**: "Unable to reach the specified URL"
- **Cause**: Network connectivity issue or inaccessible URL

### InvalidUrlError
- **Code**: INGESTION_002
- **Message**: "The provided URL is malformed or invalid"
- **Cause**: Malformed URL string provided as input

### ContentExtractionError
- **Code**: INGESTION_003
- **Message**: "Failed to extract content from the webpage"
- **Cause**: Unable to parse HTML or extract meaningful text content

### ValidationError
- **Code**: INGESTION_004
- **Message**: "Invalid parameter values provided"
- **Cause**: Parameter values do not meet validation requirements

### CohereApiError
- **Code**: INGESTION_005
- **Message**: "Error communicating with Cohere API"
- **Cause**: API is unavailable or request format is incorrect

### RateLimitError
- **Code**: INGESTION_006
- **Message**: "Rate limit exceeded for Cohere API"
- **Cause**: Too many requests to Cohere API within the allowed timeframe

### QdrantApiError
- **Code**: INGESTION_007
- **Message**: "Error communicating with Qdrant API"
- **Cause**: API is unavailable or request format is incorrect

### ConfigurationError
- **Code**: INGESTION_008
- **Message**: "Configuration parameters are invalid"
- **Cause**: Invalid values provided in configuration object

### ProcessingError
- **Code**: INGESTION_009
- **Message**: "Processing failed during pipeline execution"
- **Cause**: Unexpected error during the ingestion pipeline

## Versioning Strategy
- This API contract follows semantic versioning
- Breaking changes to the interface will increment the major version
- Backward-compatible additions will increment the minor version
- Bug fixes will increment the patch version
- Current version: 1.0.0

## Compliance
This API contract ensures compliance with the following requirements from the feature specification:
- FR-001: System extracts content from all publicly accessible pages of a provided Docusaurus website URL
- FR-002: System cleans extracted text to remove navigation, headers, footers, and other non-content elements
- FR-003: System deterministically chunks cleaned text while preserving semantic coherence
- FR-004: System generates semantic embeddings for each text chunk using Cohere embedding models
- FR-005: System persists embeddings and associated metadata to Qdrant vector database
- FR-006: System provides configurable parameters for text cleaning, chunk size, and overlap
- FR-007: System maintains source URL metadata and content timestamps with each vector entry
- FR-008: System handles failed API requests to Cohere and retries with exponential backoff