"""Test script to validate Qdrant credentials."""

import requests
import json

# Your Qdrant details
qdrant_url = "https://16d81df7-6903-4b40-8490-a7413250cf8d.europe-west3-0.gcp.cloud.qdrant.io:6333"
qdrant_api_key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.hDneFjuBDjjADxkAPnzE71ddY3Q4DwfOBKAafhvXsXQ"

print("Testing Qdrant connection...")
print(f"URL: {qdrant_url}")

headers = {
    "api-key": qdrant_api_key,
    "Content-Type": "application/json"
}

try:
    # Test connection by getting collections
    response = requests.get(f"{qdrant_url}/collections", headers=headers, timeout=10)
    
    print(f"Response Status Code: {response.status_code}")
    print(f"Response Text: {response.text[:200]}...")
    
    if response.status_code == 200:
        print("\n✅ Qdrant connection successful!")
        collections = response.json()
        print(f"Available collections: {collections}")
    elif response.status_code == 403:
        print("\n❌ Qdrant connection failed - Forbidden (403)")
        print("This means your API key is invalid or has expired.")
        print("Please check your Qdrant Cloud dashboard for a new API key.")
    elif response.status_code == 404:
        print("\n❌ Qdrant connection failed - Not Found (404)")
        print("Please check your Qdrant URL.")
    else:
        print(f"\n❌ Qdrant connection failed with status code: {response.status_code}")
        
except requests.exceptions.ConnectionError:
    print("\n❌ Could not connect to Qdrant - Connection Error")
    print("Please check your Qdrant URL and network connection.")
except requests.exceptions.Timeout:
    print("\n❌ Qdrant connection timed out")
    print("Please check your Qdrant URL and network connection.")
except Exception as e:
    print(f"\n❌ Error testing Qdrant connection: {e}")