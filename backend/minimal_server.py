"""Minimal server startup script that avoids Qdrant connection on startup."""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config.settings import settings

# Create a minimal app without the routes that might trigger Qdrant connection
app = FastAPI(
    title="Minimal RAG API",
    version="1.0.0",
    debug=True
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "Minimal API running", "status": "ok"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "api_keys_configured": bool(settings.cohere_api_key and settings.qdrant_api_key)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )