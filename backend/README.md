# RAG Ingestion Pipeline

This backend service extracts content from deployed Docusaurus websites, generates embeddings using Cohere, and stores them in Qdrant Cloud for RAG-based retrieval.

## Setup

1. Install dependencies:
   ```bash
   cd backend
   uv pip install -r requirements.txt
   ```

2. Create a `.env` file with your configuration:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_COLLECTION_NAME=book_content_chunks_v1
   SOURCE_BASE_URL=https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/
   ```

## Usage

To run the ingestion pipeline:

```bash
cd backend
python -m app.main
```

## Configuration

The following environment variables can be configured:

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: book_content_chunks_v1)
- `SOURCE_BASE_URL`: URL of the Docusaurus site to crawl (default: the textbook site)
- `CHUNK_SIZE`: Maximum size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between consecutive chunks (default: 50)
- `BATCH_SIZE`: Number of chunks to process in each batch (default: 10)
- `EMBEDDING_MODEL`: Cohere model to use for embeddings (default: embed-multilingual-v3.0)
- `MAX_RETRIES`: Maximum number of retries for failed operations (default: 3)
- `DELAY_BASE`: Base delay for exponential backoff (default: 1.0)
- `TIMEOUT`: Timeout for API requests in seconds (default: 30)

## Architecture

The pipeline consists of the following services:

- `src/rag/ingestion.py`: Main IngestionPipeline class with WebCrawler, ContentChunker, EmbeddingGenerator, and QdrantStorage (Primary recommended approach)
- `app/services/url_crawler.py`: Discovers all accessible URLs from the target site
- `app/services/text_extractor.py`: Extracts clean text content from web pages
- `app/services/chunker.py`: Splits text into overlapping chunks
- `app/services/embedding_service.py`: Generates embeddings using Cohere API
- `app/services/vector_storage.py`: Stores embeddings in Qdrant Cloud
- `app/main.py`: Legacy orchestration approach (use src/rag/ingestion.py for new implementations)

## Primary Ingestion Pipeline

The recommended ingestion pipeline is located in `src/rag/ingestion.py` and can be used as follows:

```python
from src.rag.ingestion import IngestionPipeline
from src.config.settings import settings

ingestion_pipeline = IngestionPipeline(
    cohere_api_key=settings.cohere_api_key,
    qdrant_url=settings.qdrant_url,
    qdrant_api_key=settings.qdrant_api_key,
    collection_name=settings.qdrant_collection_name
)

ingestion_pipeline.run_ingestion(
    urls=["https://your-site.com/"],
    chunk_size=512,
    overlap=50
)
```

## Testing

Run the tests with:

```bash
cd backend
pytest tests/ -v
```