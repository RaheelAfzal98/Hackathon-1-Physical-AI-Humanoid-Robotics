# Research Findings: RAG Ingestion Pipeline

## Overview
This document captures research findings and decisions for implementing the RAG ingestion pipeline that extracts book content from deployed website URLs, generates embeddings using Cohere, and stores them in Qdrant Cloud for RAG-based retrieval.

## Technology Stack Research

### Chunk Size and Overlap Trade-offs
- **Decision**: Use 512-token chunks with 50-token overlap for optimal balance between context retention and retrieval precision
- **Rationale**: Based on research, 512-token chunks work well for technical documentation and allow sufficient context for semantic search. The 50-token overlap ensures continuity across chunk boundaries while avoiding excessive redundancy.
- **Alternatives considered**: 
  - Larger chunks (1024+) could lose semantic focus
  - Smaller chunks (256-) could fragment context too much
  - No overlap could miss relevant context spanning boundaries

### Cohere Embedding Model Selection
- **Decision**: Use Cohere's `embed-multilingual-v3.0` model with text type
- **Rationale**: This model performs exceptionally well for technical documentation and supports multilingual content. It's specifically optimized for retrieval use cases and offers competitive performance on free tier.
- **Alternatives considered**:
  - OpenAI's text-embedding-ada-002 (would require separate billing)
  - Sentence-transformers models (require hosting/embedding infrastructure)
  - Hugging Face models (larger computational requirements)

### Qdrant Collection Naming and Versioning Strategy
- **Decision**: Use collection name format: `{prefix}_{content_type}_v{version}` (e.g., `book_content_chunks_v1`)
- **Rationale**: This allows for versioning of embeddings as models improve or content changes. The prefix helps organize multiple collections if needed for different content types.
- **Alternatives considered**:
  - Simple names without versioning (makes updates difficult)
  - Date-based naming (harder to track versions systematically)

### ID Generation Strategy for Idempotent Ingestion
- **Decision**: Use SHA256 hash of URL + content snippet + chunk index as document ID
- **Rationale**: This creates deterministic IDs that ensure idempotent ingestion operations. If the same content is ingested multiple times, it will have the same ID and can be updated rather than duplicated.
- **Alternatives considered**:
  - UUIDs (would create duplicates on repeated ingestion)
  - Simple sequential IDs (not suitable for distributed/parallel processing)

### Handling Updates to Existing Website Content
- **Decision**: Implement content change detection by comparing content hashes
- **Rationale**: Before ingesting, we'll hash the extracted content and compare with existing vectors in Qdrant. If the content has changed, update the vector; if unchanged, skip ingestion.
- **Alternatives considered**:
  - Always overwrite (wasteful and increases API costs)
  - Manual content change detection (impractical for large sites)

## Best Practices Implementation

### Text Cleaning for Technical Documentation
- **Research**: HTML parsing with BeautifulSoup is the standard approach for removing navigation, headers, footers, and non-content elements
- **Strategy**: Use CSS selectors to identify and remove common structural elements, then extract semantic content from articles, sections, and paragraphs
- **Considerations**: Docusaurus sites typically have consistent structures with `<main>`, `<article>`, and `<div class="theme-doc-markdown">` elements

### URL Discovery and Crawling Strategy
- **Research**: Two primary approaches for crawling Docusaurus sites: sitemap.xml parsing or recursive crawling
- **Strategy**: Prioritize sitemap.xml if available to get complete site structure, with fallback to recursive crawling limiting depth to prevent infinite loops
- **Considerations**: Need to respect robots.txt and implement rate limiting to be a good web citizen

### Error Handling and Retry Logic
- **Research**: Exponential backoff with jitter is the standard approach for API rate limiting and temporary failures
- **Strategy**: Implement 3-5 retries with 1s-30s exponential backoff with jitter for Cohere API calls and Qdrant operations
- **Considerations**: Need to distinguish between retryable (network timeouts, rate limits) and non-retryable errors (authentication failures, invalid inputs)

### Monitoring and Progress Tracking
- **Research**: Standard practice is to log progress at regular intervals and expose metrics
- **Strategy**: Log progress after each 50-100 documents processed, including success/failure rates and timing statistics
- **Considerations**: These logs will feed into dashboards and alerting systems for operational visibility

## Implementation Notes

### Dependencies Justification
- `requests`: Standard for HTTP requests with excellent error handling
- `beautifulsoup4`: Industry standard for parsing HTML and extracting content
- `cohere`: Official client for Cohere's embedding models
- `qdrant-client`: Official client for Qdrant vector database
- `python-dotenv`: Standard for managing environment variables securely
- `lxml`: High-performance XML/HTML parser for BeautifulSoup

### Performance Considerations
- Process URLs in batches to manage memory usage
- Use connection pooling for HTTP requests
- Batch vector uploads to Qdrant to reduce API calls
- Cache content hashes to avoid recalculating for repeated runs

### Architecture Patterns
- Follow service-oriented architecture with separation of concerns
- Use factory patterns for creating embeddings to allow model switching
- Implement repository pattern for vector storage abstraction

## Conclusion
All major technical decisions have been researched and documented. The implementation will follow best practices for each component while maintaining compatibility with the free tiers of Cohere and Qdrant.