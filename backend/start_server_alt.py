"""Script to start the backend server on an alternative port."""

import os
import sys
from contextlib import redirect_stderr
import io

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from src.config.settings import settings
from src.main import create_app
import uvicorn

def check_qdrant_connection():
    """Check if Qdrant is accessible and working."""
    from qdrant_client import QdrantClient
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=5
        )
        collections = client.get_collections()
        return True
    except Exception as e:
        print(f"Qdrant connection failed: {e}")
        return False

def main():
    print("Starting backend server...")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Cohere API Key configured: {'Yes' if settings.cohere_api_key else 'No'}")

    # Check Qdrant connection
    qdrant_ok = check_qdrant_connection()
    if not qdrant_ok:
        print("\n⚠️  Warning: Qdrant connection failed!")
        print("The RAG system will not work until you provide valid Qdrant credentials.")
        print("However, the server will still start for other functionality.\n")

    # Create the app
    app = create_app()

    port = int(os.environ.get("PORT", 8001))  # Use PORT env var or default to 8001
    print(f"Starting server on http://0.0.0.0:{port}")
    print("Press Ctrl+C to stop the server")

    # Start the server
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=port,
        reload=False  # Set to False to avoid issues in some environments
    )

if __name__ == "__main__":
    main()