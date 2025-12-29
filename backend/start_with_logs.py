"""Script to start the backend server and show any errors."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

try:
    from src.config.settings import settings
    print(f"Loaded settings successfully")
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Qdrant API Key available: {'Yes' if settings.qdrant_api_key else 'No'}")
    print(f"Cohere API Key available: {'Yes' if settings.cohere_api_key else 'No'}")
    
    from src.main import create_app
    print("Creating app...")
    app = create_app()
    print("App created successfully")
    
    import uvicorn
    print("Starting server on port 8000...")
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()