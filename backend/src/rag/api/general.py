"""General API endpoints for the agent system."""

from fastapi import APIRouter, HTTPException
import time
from datetime import datetime
from typing import Dict
from .query_router import router as query_router
from .session_router import router as session_router
from .config_router import router as config_router


router = APIRouter()

# Include the specific routers
router.include_router(query_router)
router.include_router(session_router)
router.include_router(config_router)


@router.get("/health", response_model=Dict)
async def health_check():
    """Health check endpoint to verify the agent system is operational."""
    try:
        # Perform basic checks
        status = {
            "status": "healthy",
            "service": "agent-rag-service",
            "timestamp": datetime.utcnow().isoformat(),
            "checks": {
                "api_access": "ok",
                "dependencies": "pending",  # More detailed dependency checks would go here
            }
        }
        return status
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e),
            "timestamp": datetime.utcnow().isoformat()
        }


@router.get("/stats", response_model=Dict)
async def get_system_stats():
    """Get system statistics and performance metrics."""
    try:
        stats = {
            "uptime_seconds": time.time(),  # This would be calculated properly in a real implementation
            "total_requests": 0,  # Would track actual requests in a real implementation
            "average_response_time": 0,  # Would track actual response times
            "active_sessions": 0,  # Would track active sessions
            "timestamp": datetime.utcnow().isoformat()
        }
        return stats
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving stats: {str(e)}")


@router.get("/")
async def root():
    """Root endpoint with basic information about the service."""
    return {
        "service": "Agentic RAG Backend",
        "description": "AI agent backend with RAG capabilities for documentation queries",
        "version": "1.0.0",
        "endpoints": [
            "/agent/query",
            "/agent/session",
            "/agent/config",
            "/agent/health"
        ]
    }