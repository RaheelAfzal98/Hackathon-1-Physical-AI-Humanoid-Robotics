"""Test script to check Qdrant client methods."""

from qdrant_client import QdrantClient

# Create an instance to check available methods (without connecting)
client = QdrantClient(url="http://localhost:6333", https=False)

# Look for search-related methods
methods = [method for method in dir(client) if 'search' in method.lower()]
print("Available search methods:", methods)

# Check for the actual search method used in Qdrant
for method in methods:
    print(f"Method: {method}")
    print(f"  Docstring: {getattr(client, method).__doc__[:200] if getattr(client, method).__doc__ else 'No docstring'}")