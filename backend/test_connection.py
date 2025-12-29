import os
from qdrant_client import QdrantClient

# Get config from environment
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

print(f"Testing connection to Qdrant at: {qdrant_url}")

try:
    # Initialize client
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        https=True
    )
    
    # Try to list collections to test the connection
    collections = client.get_collections()
    print(f"✅ Successfully connected! Found {len(collections.collections)} collections")
    
    # Print collection names
    for collection in collections.collections:
        print(f"  - {collection.name}")
        
except Exception as e:
    print(f"Connection failed: {e}")

    # Let's also test with the port explicit
    try:
        # Extract host from URL (remove protocol and port)
        import re
        host_match = re.match(r'https://([^:/]+)', qdrant_url)
        if host_match:
            host = host_match.group(1)
            print(f"Trying with separate host: {host} and default port 6333...")

            client = QdrantClient(
                host=host,
                api_key=qdrant_api_key,
                https=True,
                port=6333
            )

            collections = client.get_collections()
            print(f"Successfully connected with separate host! Found {len(collections.collections)} collections")
    except Exception as e2:
        print(f"Connection with separate host also failed: {e2}")

print("Done!")