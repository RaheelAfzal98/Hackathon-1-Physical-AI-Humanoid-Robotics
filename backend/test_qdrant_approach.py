"""Test script to check the exact way to use Qdrant search in our environment."""

# Try the approach that should work based on the Qdrant documentation
from qdrant_client import QdrantClient
from qdrant_client.http import models

print("Creating Qdrant client...")
try:
    # This doesn't connect initially, just configures the client
    client = QdrantClient(
        url="https://test-url.qdrant.io",  # Using a test URL
        api_key="test-key"
    )
    print(f"Client type: {type(client)}")
    
    # Test what methods are available
    methods = [m for m in dir(client) if not m.startswith('_')]
    search_methods = [m for m in methods if 'search' in m.lower()]
    print(f"Available search-like methods: {search_methods}")
    
    # Check if the client has an 'http' attribute that might access the HTTP API
    if hasattr(client, '_client') or hasattr(client, '_rest'):
        if hasattr(client, '_client'):  # For internal client access
            internal_client_methods = [m for m in dir(client._client) if 'search' in m.lower()]
            print(f"Internal client search methods: {internal_client_methods}")
        if hasattr(client, '_rest'):
            rest_client_methods = [m for m in dir(client._rest) if 'search' in m.lower()]
            print(f"Rest client search methods: {rest_client_methods}")
            
    # Try another approach - perhaps it's an asynchronous client issue
    # Check if we need to access the points API directly somehow
    print("Checking for PointsAPI access...")
    try:
        if hasattr(client, 'async_points_api') or hasattr(client, 'points_api'):
            print(f"Has points_api: {hasattr(client, 'points_api')}")
            print(f"Has async_points_api: {hasattr(client, 'async_points_api')}")
    except:
        pass
    
    # Try to see if search is accessible in a different way
    # Let's look at all methods from the actual client implementation
    all_public_methods = [m for m in dir(client) if not m.startswith('_')]
    print(f"First 20 public methods: {all_public_methods[:20]}")
    print(f"Has search: {'search' in dir(client)}")
    
except Exception as e:
    print(f"Error creating client: {e}")
    

# Let's also try connecting with a real client temporarily to see what methods are available after initialization
print("\nTrying to inspect without connecting first...")
# Check the class methods again
from qdrant_client.qdrant_client import QdrantClient as RealClient
import inspect

# Get all methods from the class
cls = RealClient
all_cls_methods = [m for m in dir(cls) if not m.startswith('_')]
search_cls_methods = [m for m in all_cls_methods if 'search' in m.lower()]
print(f"Class methods related to search: {search_cls_methods}")

# Check if there are any private methods that might be wrappers
private_methods = [m for m in dir(cls) if m.startswith('_') and not m.startswith('__')]
private_search_methods = [m for m in private_methods if 'search' in m.lower()]
print(f"Private methods related to search: {private_search_methods}")

# Let's try a direct import from the http models to see what's available
print("\nTrying to access the underlying http API...")
from qdrant_client.http import api
print(f"Available APIs in qdrant_client.http.api: {[x for x in dir(api) if not x.startswith('_')]}")