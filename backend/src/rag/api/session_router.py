"""Session router for the agent API endpoints."""

from fastapi import APIRouter, HTTPException, status
from typing import List
from ..models.session import AgentSession
from ..services.session_manager import SessionManager
from ..config.agent_config import AgentConfig


router = APIRouter(prefix="/session", tags=["session"])


# Global session manager instance
# In a real application, you would inject this dependency properly
agent_config = AgentConfig()
session_manager = SessionManager(agent_config)


@router.post("/", response_model=dict)
async def create_session(config: dict = None):
    """Create a new agent session for maintaining conversation context."""
    try:
        from ..models.configuration import AgentConfiguration
        
        # Convert config dict to AgentConfiguration object if provided
        agent_config = None
        if config:
            agent_config = AgentConfiguration(**config)
        
        session_id = session_manager.create_session(agent_config)
        
        session = session_manager.get_session(session_id)
        
        return {
            "session_id": session_id,
            "created_at": session.created_at,
            "config": session.config.dict()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create session: {str(e)}")


@router.post("/{session_id}/query", response_model=dict)
async def query_in_session(session_id: str, query_data: dict):
    """Query the agent within a session context, maintaining conversation history."""
    try:
        from ..models.query_request import QueryRequest
        from ..models.agent_response import AgentResponse
        
        # Validate session exists
        session = session_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        # Create a QueryRequest from the query_data
        query_request = QueryRequest(**query_data)
        
        # Process the query in session context
        # For now, returning a dummy response - in a complete implementation,
        # we'd integrate with the agent query service to handle session context
        
        return session
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=f"Invalid query data: {str(ve)}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to process session query: {str(e)}")


@router.get("/", response_model=List[dict])
async def list_sessions():
    """List active sessions for monitoring purposes."""
    try:
        sessions_info = session_manager.list_sessions()
        return sessions_info
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to list sessions: {str(e)}")


@router.delete("/{session_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_session(session_id: str):
    """Delete a session and clean up resources."""
    try:
        success = session_manager.delete_session(session_id)
        if not success:
            raise HTTPException(status_code=404, detail="Session not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to delete session: {str(e)}")


@router.get("/{session_id}/config", response_model=dict)
async def get_session_config(session_id: str):
    """Retrieve the configuration for a specific session."""
    try:
        from ..models.configuration import AgentConfiguration
        
        session = session_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        return {"config": session.config.dict()}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get session config: {str(e)}")


@router.put("/{session_id}/config", response_model=dict)
async def update_session_config(session_id: str, config: dict):
    """Update the configuration for a specific session."""
    try:
        from ..models.configuration import AgentConfiguration
        
        # Validate the incoming config
        agent_config = AgentConfiguration(**config)
        
        success = session_manager.update_session_config(session_id, agent_config)
        if not success:
            raise HTTPException(status_code=404, detail="Session not found")
        
        # Return the updated config
        updated_session = session_manager.get_session(session_id)
        return {"config": updated_session.config.dict()}
    except ValueError as ve:
        raise HTTPException(status_code=400, detail=f"Invalid configuration: {str(ve)}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update session config: {str(e)}")