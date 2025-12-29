import os
import sys
import traceback
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
from qdrant_client import QdrantClient

print('FINAL VERIFICATION: Checking stored content in Qdrant...')

try:
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        https=True,
        timeout=10
    )
    
    # Check if our collection exists now with the new name
    collection_info = client.get_collection(settings.qdrant_collection_name)
    print(f'Collection {settings.qdrant_collection_name} exists!')
    print(f'Total stored embeddings: {collection_info.points_count}')
    print(f'Vector dimensions: {collection_info.config.params.size}')
    
    # Sample a few points if there are any
    if collection_info.points_count > 0:
        # Get a few sample points
        sample_points = client.scroll(
            collection_name=settings.qdrant_collection_name,
            limit=3
        )
        print('\nSample of stored content:')
        for i, point in enumerate(sample_points[0][:3]):  # Show first 3 points
            print(f'  Point {i+1}:')
            print(f'    ID: {point.id}')
            content_snippet = point.payload.get('content', '')[:100] + '...' if len(point.payload.get('content', '')) > 100 else point.payload.get('content', '')
            print(f'    Content: {content_snippet}')
            print(f'    Source: {point.payload.get("metadata", {}).get("source_url", "N/A")}')
            print(f'    Title: {point.payload.get("metadata", {}).get("page_title", "N/A")}')
        
        print(f'\nSUCCESS: Your AI book content has been successfully stored in Qdrant!')
        print(f'The embeddings are ready for semantic search and retrieval.')
        print(f'Total points stored: {collection_info.points_count}')
    else:
        print('No points found in collection - ingestion may not have completed properly')
        
except Exception as e:
    print(f'Error during verification: {str(e)}')
    traceback.print_exc()