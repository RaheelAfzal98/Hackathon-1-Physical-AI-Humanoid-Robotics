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

print(f'Checking if the {settings.qdrant_collection_name} was created and populated...')

try:
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        https=True,
        timeout=10
    )
    
    # First, list all collections
    collections = client.get_collections()
    print(f'Available collections: {[coll.name for coll in collections.collections]}')
    
    # Check if our collection exists now
    try:
        collection_info = client.get_collection(settings.qdrant_collection_name)
        print(f'SUCCESS: Collection {settings.qdrant_collection_name} now exists!')
        print(f'Points in collection: {collection_info.points_count}')
        print(f'Vector size: {collection_info.config.params.size}')
        
        # Sample a few points if there are any
        if collection_info.points_count > 0:
            # Get a few sample points
            sample_points = client.scroll(
                collection_name=settings.qdrant_collection_name,
                limit=2
            )
            print('\nSample of stored points:')
            for i, point in enumerate(sample_points[0][:2]):  # Show first 2 points
                print(f'  Point {i+1}:')
                print(f'    ID: {point.id}')
                content_snippet = point.payload.get("content", "")[:100] + "..." if len(point.payload.get("content", "")) > 100 else point.payload.get("content", "")
                print(f'    Content snippet: {content_snippet}')
                print(f'    Source URL: {point.payload.get("metadata", {}).get("source_url", "N/A")}')
                print(f'    Document ID: {point.payload.get("metadata", {}).get("document_id", "N/A")}')
        else:
            print("Collection exists but may still be processing or empty")
    except Exception as e:
        print(f'Collection {settings.qdrant_collection_name} does not exist yet: {str(e)}')
        
except Exception as e:
    print(f'ERROR: Error with Qdrant connection: {str(e)}')
    traceback.print_exc()