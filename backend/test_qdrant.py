import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from qdrant_client import QdrantClient
from src.config.settings import settings

print("Testing Qdrant connection...")

try:
    # Create Qdrant client
    client = QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        timeout=10
    )
    
    # Test connection by trying to list collections
    collections = client.get_collections()
    print(f"Successfully connected to Qdrant!")
    print(f"Available collections: {[col.name for col in collections.collections]}")
    
    # Check if our collection exists
    collection_name = settings.qdrant_collection_name
    collection_exists = any(col.name == collection_name for col in collections.collections)
    
    if collection_exists:
        print(f"Collection '{collection_name}' exists")
        # Get collection info
        collection_info = client.get_collection(collection_name)
        print(f"Collection vectors count: {collection_info.points_count}")
    else:
        print(f"Collection '{collection_name}' does not exist")
        print("This might be why the RAG system is not working.")
    
except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
    import traceback
    traceback.print_exc()