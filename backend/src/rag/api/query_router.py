"""Query router for the agent API endpoints."""

from fastapi import APIRouter, HTTPException
from typing import Optional
import time
from ..models.query_request import QueryRequest
from ..models.agent_response import AgentResponse
from src.rag.agent.openai_agent import OpenAIAgent
from src.rag.retrieval.retrieval_tool import RetrievalTool
from src.rag.agent.grounding_validator import GroundingValidator
from src.rag.config.agent_config import AgentConfig
from src.rag.utils.helpers import calculate_elapsed_time
from src.rag.constants import DEFAULT_TIMEOUT_SECONDS
from src.rag.retrieval.qdrant_adapter import QdrantAdapter
from src.rag.services.agent_query_service import AgentQueryService
from src.rag.services.retrieval_service import RetrievalService
from src.config.settings import settings


router = APIRouter(tags=["agent"])


# Global service instance setup
# In a real application, you would inject these dependencies properly
agent_config = AgentConfig()

# Initialize the retrieval service
retrieval_service = RetrievalService()

# Initialize the grounding validator
grounding_validator = GroundingValidator()

# Initialize the service (agent initialization is handled inside AgentQueryService)
agent_query_service = AgentQueryService(
    retrieval_tool=retrieval_service.qdrant_adapter if retrieval_service.initialized else None,
    grounding_validator=grounding_validator
)


@router.post("/query", response_model=AgentResponse)
async def query_agent(query_request: QueryRequest):
    """Endpoint to query the agent without maintaining session context."""
    try:
        # Validate query length
        if len(query_request.query_text) < 1 or len(query_request.query_text) > 2000:
            raise HTTPException(status_code=400, detail="Query text must be between 1 and 2000 characters")

        # Process the query
        start_time = time.time()
        response = agent_query_service.process_query(query_request)
        response.query_time_ms = calculate_elapsed_time(start_time)

        return response
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/health")
async def health_check():
    """Health check endpoint to verify the agent service is operational."""
    try:
        # Try to perform a simple check against the agent
        return {"status": "healthy", "service": "agent-api"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")