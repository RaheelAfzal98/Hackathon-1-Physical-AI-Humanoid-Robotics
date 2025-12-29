import os
import sys
import traceback

# Add the necessary paths to sys.path
sys.path.append('.')
sys.path.append('src')

# Load environment variables from .env file if it exists
if os.path.exists('.env'):
    with open('.env', 'r') as file:
        for line in file:
            if line.strip() and not line.startswith('#'):
                key, value = line.strip().split('=', 1)
                os.environ[key] = value

from src.config.settings import settings
from src.rag.ingestion import IngestionPipeline

print('Starting ingestion of AI book content into Qdrant...')
print(f'Using Qdrant collection: {settings.qdrant_collection_name}')

# Define the URLs to crawl - based on your base URL
base_url = 'https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app'

# Common paths for your book/ documentation site
urls_to_crawl = [
    f'{base_url}',
]

print(f'Crawling the following URLs: {urls_to_crawl}')

try:
    # Create ingestion pipeline with your settings
    ingestion_pipeline = IngestionPipeline(
        cohere_api_key=settings.cohere_api_key,
        qdrant_url=settings.qdrant_url,
        qdrant_api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name
    )
    
    print('Starting ingestion process...')
    
    # Run the ingestion with more verbose output
    ingestion_pipeline.run_ingestion(
        urls=urls_to_crawl,
        chunk_size=512,  # Appropriate for technical content
        overlap=50
    )
    
    print('\nIngestion completed successfully!')
    print(f'Your book content has been embedded and stored in Qdrant collection: {settings.qdrant_collection_name}')
    print('You can now use the retrieval API to search through your book content.')

except Exception as e:
    print(f'Error during ingestion: {str(e)}')
    traceback.print_exc()