# Quickstart Guide: RAG Ingestion Pipeline

## Overview
This guide provides instructions to quickly set up and run the RAG ingestion pipeline that extracts book content from deployed website URLs, generates embeddings using Cohere, and stores them in Qdrant Cloud for RAG-based retrieval.

## Prerequisites

### System Requirements
- Python 3.11 or higher
- pip package manager
- UV (Python project initializer)
- Git (to clone the repository, if applicable)

### Third-party Services
- Cohere API key (free tier available)
- Qdrant Cloud account with cluster (free tier available)

## Setup Instructions

### 1. Clone and Navigate to Backend Directory
```bash
# Clone the repository (if applicable)
git clone <repository-url>
cd Hackathon-I -Create-a-Textbook

# Change to the backend directory
cd backend
```

### 2. Install UV (if not already installed)
```bash
pip install uv
```

### 3. Initialize the Python Project with UV
```bash
# Inside the backend directory
uv init
```

### 4. Install Dependencies
```bash
# Install all required packages
uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv lxml pytest
```

Alternatively, create a `requirements.txt` file with:
```
requests>=2.31.0
beautifulsoup4>=4.12.2
cohere>=4.0.0
qdrant-client>=1.9.1
python-dotenv>=1.0.0
lxml>=4.9.0
pytest>=8.0.0
```

And install using:
```bash
uv pip install -r requirements.txt
```

### 5. Set Up Environment Variables
Create a `.env` file in the backend directory with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_content_chunks_v1
SOURCE_BASE_URL=https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/
```

**Note**: Add `.env` to your `.gitignore` file to avoid committing secrets to version control.

Example `.gitignore` entry:
```
.env
__pycache__/
*.pyc
```

### 6. Create Project Structure
Set up the following directory structure in the backend folder:

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py
│   ├── models/
│   │   ├── __init__.py
│   │   └── chunk.py
│   ├── services/
│   │   ├── __init__.py
│   │   ├── url_crawler.py
│   │   ├── text_extractor.py
│   │   ├── chunker.py
│   │   ├── embedding_service.py
│   │   └── vector_storage.py
│   ├── utils/
│   │   ├── __init__.py
│   │   ├── html_cleaner.py
│   │   └── id_generator.py
│   └── config/
│       ├── __init__.py
│       └── settings.py
├── tests/
│   ├── __init__.py
│   └── test_main.py
├── pyproject.toml
├── requirements.txt
├── README.md
└── .env
```

## Running the Ingestion Pipeline

### 1. Create the Main Application File
Create `backend/app/main.py` with the following implementation:

```python
import asyncio
from typing import List, Dict, Any
from app.services.url_crawler import get_all_urls
from app.services.text_extractor import extract_text_from_url
from app.services.chunker import chunk_text
from app.services.embedding_service import embed
from app.services.vector_storage import create_collection, save_chunk_to_qdrant
from app.config.settings import settings


async def execute_ingestion_pipeline(
    base_url: str = settings.SOURCE_BASE_URL,
    collection_name: str = settings.QDRANT_COLLECTION_NAME
):
    """
    Execute the complete ingestion pipeline from URL discovery to vector storage
    """
    print(f"Starting ingestion pipeline for: {base_url}")
    
    # Step 1: Discover all accessible URLs
    urls = await get_all_urls(base_url)
    print(f"Discovered {len(urls)} URLs to process")
    
    # Step 2: Create Qdrant collection if it doesn't exist
    await create_collection(collection_name, vector_size=settings.EMBEDDING_DIMENSION)
    print(f"Collection '{collection_name}' prepared")
    
    total_chunks = 0
    processed_urls = 0
    
    # Step 3: Process each URL
    for i, url in enumerate(urls):
        print(f"Processing URL {i+1}/{len(urls)}: {url}")
        
        try:
            # Extract clean text from the URL
            text_data = await extract_text_from_url(url)
            
            # Chunk the text content
            chunks = chunk_text(
                text_data['text'],
                chunk_size=settings.CHUNK_SIZE,
                chunk_overlap=settings.CHUNK_OVERLAP
            )
            
            # Generate embeddings for each chunk
            text_list = [chunk['text'] for chunk in chunks]
            embeddings = await embed(text_list)
            
            # Store each chunk with its embedding in Qdrant
            for j, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                chunk_data = {
                    **chunk,
                    'source_url': url,
                    'title': text_data.get('title', ''),
                    'hash': text_data.get('hash', ''),
                    'created_at': text_data.get('created_at', ''),
                    'updated_at': text_data.get('updated_at', '')
                }
                
                chunk_id = await save_chunk_to_qdrant(
                    chunk_data=chunk_data,
                    collection_name=collection_name,
                    vector=embedding
                )
                
                total_chunks += 1
                print(f"  Saved chunk {j+1}/{len(chunks)} with ID: {chunk_id[:8]}...")
                
            processed_urls += 1
            
        except Exception as e:
            print(f"Error processing URL {url}: {str(e)}")
            continue
    
    stats = {
        'urls_processed': processed_urls,
        'total_urls_found': len(urls),
        'chunks_created': total_chunks,
        'collection_name': collection_name
    }
    
    print(f"\nPipeline completed!")
    print(f"Processed {processed_urls}/{len(urls)} URLs")
    print(f"Created {total_chunks} content chunks")
    print(f"All vectors stored in collection: {collection_name}")
    
    return stats


if __name__ == "__main__":
    # Run the ingestion pipeline
    stats = asyncio.run(execute_ingestion_pipeline())
    print("\nPipeline Statistics:")
    print(stats)
```

### 2. Create Configuration Settings
Create `backend/app/config/settings.py`:

```python
from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

class Settings:
    # Cohere settings
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    
    # Qdrant settings
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks_v1")
    
    # Source settings
    SOURCE_BASE_URL = os.getenv("SOURCE_BASE_URL", "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/")
    
    # Processing settings
    CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "512"))
    CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "50"))
    BATCH_SIZE = int(os.getenv("BATCH_SIZE", "10"))
    EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "embed-multilingual-v3.0")
    EMBEDDING_DIMENSION = int(os.getenv("EMBEDDING_DIMENSION", "1024"))  # For multilingual-v3.0
    
    # Retry settings
    MAX_RETRIES = int(os.getenv("MAX_RETRIES", "3"))
    DELAY_BASE = float(os.getenv("DELAY_BASE", "1.0"))
    TIMEOUT = int(os.getenv("TIMEOUT", "30"))
    
    # Validation
    def validate(self):
        errors = []
        if not self.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is required")
        if not self.QDRANT_URL:
            errors.append("QDRANT_URL is required")
        if not self.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is required")
        
        if errors:
            raise ValueError(f"Configuration errors: {'; '.join(errors)}")


settings = Settings()
settings.validate()
```

### 3. Run the Ingestion Pipeline
Execute the pipeline with:

```bash
cd backend
python -m app.main
```

## Testing the Implementation

### Run Unit Tests
```bash
cd backend
pytest tests/ -v
```

### Sample Test File
Create `backend/tests/test_main.py`:

```python
import pytest
from unittest.mock import AsyncMock, patch
from app.main import execute_ingestion_pipeline


@pytest.mark.asyncio
async def test_execute_ingestion_pipeline():
    with patch('app.main.get_all_urls', new_callable=AsyncMock) as mock_get_urls, \
         patch('app.main.create_collection', new_callable=AsyncMock) as mock_create_collection, \
         patch('app.main.extract_text_from_url', new_callable=AsyncMock) as mock_extract_text, \
         patch('app.main.chunk_text') as mock_chunk_text, \
         patch('app.main.embed', new_callable=AsyncMock) as mock_embed, \
         patch('app.main.save_chunk_to_qdrant', new_callable=AsyncMock) as mock_save_chunk:
        
        # Mock return values
        mock_get_urls.return_value = ["https://example.com/page1"]
        mock_extract_text.return_value = {
            'text': 'Sample content for testing',
            'title': 'Test Page',
            'hash': 'abc123'
        }
        mock_chunk_text.return_value = [{
            'text': 'Sample content for testing',
            'chunk_index': 0,
            'total_chunks': 1
        }]
        mock_embed.return_value = [[0.1, 0.2, 0.3]]
        mock_save_chunk.return_value = "test-id-123"
        
        # Execute the pipeline
        result = await execute_ingestion_pipeline(
            base_url="https://example.com",
            collection_name="test_collection"
        )
        
        # Assertions
        assert result['urls_processed'] == 1
        assert result['chunks_created'] == 1
        assert result['collection_name'] == "test_collection"
        
        # Verify mocks were called
        mock_get_urls.assert_called_once()
        mock_create_collection.assert_called_once()
        mock_extract_text.assert_called_once()
        mock_chunk_text.assert_called_once()
        mock_embed.assert_called_once()
        mock_save_chunk.assert_called_once()
```

## Verification Steps

1. Verify that the Qdrant collection has been populated with vectors
2. Test that the pipeline can handle your specific Docusaurus site structure
3. Confirm that embeddings are being generated correctly
4. Validate that all source URLs are properly preserved in the metadata

## Troubleshooting

### Common Issues:
1. **Authentication errors**: Verify API keys are correct and have sufficient permissions
2. **Network errors**: Check that the target website is accessible
3. **Parsing errors**: If content isn't extracted properly, check HTML structure assumptions
4. **Rate limiting**: If hitting API limits, adjust the delay settings in the configuration

### Getting Help:
- Check the logs for detailed error messages
- Verify your Cohere and Qdrant service statuses
- Ensure your environment variables are properly set