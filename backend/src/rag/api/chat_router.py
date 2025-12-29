"""Chat router for the agent API endpoints that handles conversational queries."""

from fastapi import APIRouter, HTTPException
from typing import Optional
import time
from ..models.query_request import QueryRequest
from ..models.agent_response import AgentResponse
from src.rag.services.agent_query_service import AgentQueryService
from src.rag.services.retrieval_service import RetrievalService
from src.rag.agent.grounding_validator import GroundingValidator
from src.rag.utils.helpers import calculate_elapsed_time
from src.config.settings import settings


router = APIRouter(tags=["chat"])


# Initialize services - using the same pattern as query_router.py
agent_config = None  # Will be created inside AgentQueryService if needed

# Initialize the retrieval service
retrieval_service = RetrievalService()

# Initialize the grounding validator
grounding_validator = GroundingValidator()

# Initialize the service (agent initialization is handled inside AgentQueryService)
agent_query_service = AgentQueryService(
    retrieval_tool=retrieval_service.qdrant_adapter if retrieval_service.initialized else None,
    grounding_validator=grounding_validator
)


@router.post("/chat", response_model=AgentResponse)
async def chat_with_agent(query_request: QueryRequest):
    """
    Enhanced chat endpoint that handles conversational queries about book-related topics.
    This endpoint is optimized for natural language interactions and textbook content.
    """
    try:
        # Validate query length
        if len(query_request.query_text) < 1 or len(query_request.query_text) > 2000:
            raise HTTPException(
                status_code=400,
                detail="Query text must be between 1 and 2000 characters"
            )

        # Process the query using the existing agent query service
        start_time = time.time()
        response = agent_query_service.process_query(query_request)
        response.query_time_ms = calculate_elapsed_time(start_time)

        return response
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/chat/conversation", response_model=AgentResponse)
async def chat_in_conversation(query_request: QueryRequest):
    """
    Chat endpoint that maintains conversation context for more natural interactions.
    """
    try:
        # Validate query length
        if len(query_request.query_text) < 1 or len(query_request.query_text) > 2000:
            raise HTTPException(
                status_code=400,
                detail="Query text must be between 1 and 2000 characters"
            )

        # Process the query with conversation context
        start_time = time.time()

        # If session_id is provided, we could maintain conversation history
        # For now, using the same service but with potential for session context
        response = agent_query_service.process_query(query_request)
        response.query_time_ms = calculate_elapsed_time(start_time)

        return response
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/chat/health")
async def chat_health_check():
    """Health check endpoint for the chat service."""
    try:
        return {
            "status": "healthy",
            "service": "chat-api",
            "capabilities": [
                "Natural language query processing",
                "Book topic understanding",
                "Conversational responses",
                "Context-aware answers"
            ]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")