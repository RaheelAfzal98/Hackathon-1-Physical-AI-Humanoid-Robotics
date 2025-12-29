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


app = create_app()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )