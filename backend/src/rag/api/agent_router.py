"""API Router for the agent-based RAG functionality."""

from fastapi import APIRouter
from .query_router import router as query_router
from .session_router import router as session_router
from .config_router import router as config_router
from .health import router as health_router
from .chat_router import router as chat_router


# Create the main agent router
router = APIRouter()

# Include all the sub-routers under the agent router
router.include_router(query_router)
router.include_router(session_router)
router.include_router(config_router)
router.include_router(health_router)
router.include_router(chat_router)


# Additional endpoints specific to agent operations
@router.get("/status")
async def agent_status():
    """Get the status of the agent system."""
    return {
        "status": "operational",
        "service": "agentic-rag-agent",
        "version": "1.0.0"
    }


@router.get("/capabilities")
async def agent_capabilities():
    """Get the capabilities of the agent system."""
    return {
        "capabilities": [
            "Natural language query processing",
            "Semantic content retrieval",
            "Grounded response generation",
            "Session-based conversations",
            "Configurable behavior parameters",
            "Source citation generation"
        ]
    }