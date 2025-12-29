"""Script to test the proper Qdrant API usage with cloud instance."""

from qdrant_client import QdrantClient

print("Testing QdrantClient capabilities...")

# Check if search method exists on the class
client_class_methods = [m for m in dir(QdrantClient) if 'search' in m.lower()]
print(f"QdrantClient class methods with 'search': {client_class_methods}")

# Check if search method is available on an instance
try:
    # Create client without connecting (using dummy URL to avoid actual connection)
    client = QdrantClient(
        url="https://dummy-url.qdrant.io",
        api_key="dummy-key"
    )
    
    # Check methods on the instance
    instance_methods = [m for m in dir(client) if 'search' in m.lower()]
    print(f"QdrantClient instance methods with 'search': {instance_methods}")
    
    # Check if 'search' is directly callable
    has_search = hasattr(client, 'search')
    print(f"Has 'search' method: {has_search}")
    
except Exception as e:
    print(f"Could not create client instance: {e}")
    # Even if we can't instantiate, let's check the class methods
    print("Class methods that might be available after connection setup:")
    all_methods = [m for m in dir(QdrantClient) if not m.startswith('_')]
    search_like = [m for m in all_methods if 'search' in m.lower() or 'Search' in m.lower()]
    print(search_like)