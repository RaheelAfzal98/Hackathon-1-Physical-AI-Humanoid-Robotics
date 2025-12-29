import os
import sys
from qdrant_client import QdrantClient

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

# Initialize Qdrant client
client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
    https=True if "https" in settings.qdrant_url else False
)

print(f"Validating data in Qdrant collection: {settings.qdrant_collection_name}")

try:
    # Get collection info
    collection_info = client.get_collection(settings.qdrant_collection_name)
    print(f"Collection exists: {settings.qdrant_collection_name}")
    print(f"Points count: {collection_info.points_count}")
    print(f"Vector size: {collection_info.config.params.vectors.size}")
    print(f"Distance: {collection_info.config.params.vectors.distance}")

    # Fetch a few points to verify content
    if collection_info.points_count > 0:
        # Get 3 random points to validate content
        scroll_result = client.scroll(
            collection_name=settings.qdrant_collection_name,
            limit=3
        )

        print(f"\nSample points from collection:")
        for i, point in enumerate(scroll_result[0]):
            print(f"\nPoint {i+1}:")
            print(f"  ID: {point.id}")
            print(f"  Source URL: {point.payload.get('source_url', 'N/A')}")
            print(f"  Page Title: {point.payload.get('page_title', 'N/A')}")
            print(f"  Section Heading: {point.payload.get('section_heading', 'N/A')}")
            print(f"  Chunk Index: {point.payload.get('chunk_index', 'N/A')}")
            print(f"  Content Type: {point.payload.get('content_type', 'N/A')}")
            content_preview = point.payload.get('content', '')[:100] + "..." if len(point.payload.get('content', '')) > 100 else point.payload.get('content', '')[:100]
            print(f"  Content Preview: {content_preview}")

    print(f"\nValidation completed successfully!")
    print(f"Collection {settings.qdrant_collection_name} is ready for semantic search and RAG.")

except Exception as e:
    print(f"Error validating collection: {e}")