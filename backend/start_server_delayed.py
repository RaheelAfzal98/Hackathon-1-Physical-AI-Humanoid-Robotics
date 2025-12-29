"""Modified main.py that delays Qdrant connection until it's actually needed."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config.settings import settings
from src.api.routes import rag
from src.rag.api.agent_router import router as agent_router  # Import the agent router


def create_app():
    app = FastAPI(
        title=settings.app_name,
        version=settings.version,
        debug=settings.debug
    )

    # Add CORS middleware to allow frontend connections
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, replace with specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routes
    app.include_router(rag.router, prefix="/rag", tags=["rag"])

    # Include agent API routes
    app.include_router(agent_router, prefix="/agent", tags=["agent"])

    @app.get("/")
    def read_root():
        return {"message": "RAG Retrieval API", "version": settings.version}

    return app


# Create app instance
app = create_app()


if __name__ == "__main__":
    import uvicorn
    # Start with reload=False to avoid potential issues with the Qdrant connection during reload
    uvicorn.run(
        "start_server_delayed:app",
        host="0.0.0.0",
        port=8000,
        reload=False
    )