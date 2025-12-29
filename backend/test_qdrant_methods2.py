"""Test script to check Qdrant client methods properly."""

# This imports the correct client that was working before
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Create a client without connecting to test available methods
client_methods = [method for method in dir(QdrantClient) if not method.startswith('_')]
search_methods = [method for method in client_methods if 'search' in method.lower() or 'Search' in method]

print("All search-related methods:", search_methods)

# Also check if the async client has different methods
from qdrant_client.http.apis import PointsApi
points_api = PointsApi
api_methods = [method for method in dir(points_api) if not method.startswith('_')]
api_search_methods = [method for method in api_methods if 'search' in method.lower()]

print("API search methods:", api_search_methods)

# Let me try with a different approach - import the async client
from qdrant_client.async_qdrant_client import AsyncQdrantClient
async_client = AsyncQdrantClient
async_methods = [method for method in dir(async_client) if 'search' in method.lower()]
print("Async client search methods:", async_methods)