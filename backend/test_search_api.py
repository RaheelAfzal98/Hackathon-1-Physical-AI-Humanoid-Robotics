"""Testing the HTTP API approach for Qdrant search."""

from qdrant_client.http import api
from qdrant_client.http import models

# Check what methods are available in the search_api
search_api_class = api.search_api.SyncSearchApi
methods = [m for m in dir(search_api_class) if not m.startswith('_')]
search_methods = [m for m in methods if 'search' in m.lower() or 'Search' in m.lower()]
print(f"Search API methods: {search_methods}")

# Check the main search method if it exists
if 'search' in methods or 'search_points' in methods:
    if 'search_points' in methods:
        print("Using search_points method")
        method_name = 'search_points'
    elif 'search' in methods:
        print("Using search method")  
        method_name = 'search'
    else:
        print(f"No direct search method found, available: {methods}")
else:
    print(f"No search-related methods found in SearchApi, available: {methods}")