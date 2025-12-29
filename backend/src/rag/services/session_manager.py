"""Session management service for maintaining conversation context."""

import uuid
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Tuple
from ..models.session import AgentSession
from ..models.configuration import AgentConfiguration
from .agent_query_service import AgentQueryService
from ..config.agent_config import AgentConfig


class SessionManager:
    """Service to manage agent sessions and conversation context."""
    
    def __init__(self, agent_config: AgentConfig):
        """Initialize the session manager."""
        self.sessions: Dict[str, AgentSession] = {}
        self.agent_config = agent_config
        
    def create_session(self, config: Optional[AgentConfiguration] = None) -> str:
        """Create a new session and return the session ID."""
        session_id = str(uuid.uuid4())
        
        # Use provided config or default
        session_config = config or AgentConfiguration()
        
        session = AgentSession(
            session_id=session_id,
            created_at=datetime.now(),
            config=session_config,
            metadata={}
        )
        
        self.sessions[session_id] = session
        return session_id
    
    def get_session(self, session_id: str) -> Optional[AgentSession]:
        """Retrieve a session by its ID."""
        return self.sessions.get(session_id)
    
    def update_session(self, session_id: str, updates: Dict) -> bool:
        """Update an existing session with new data."""
        if session_id not in self.sessions:
            return False
        
        session = self.sessions[session_id]
        
        # Update fields based on the updates dict
        for key, value in updates.items():
            if hasattr(session, key):
                setattr(session, key, value)
        
        # Update the last interaction time
        session.last_interaction = datetime.now()
        self.sessions[session_id] = session
        
        return True
    
    def delete_session(self, session_id: str) -> bool:
        """Delete a session."""
        if session_id in self.sessions:
            del self.sessions[session_id]
            return True
        return False
    
    def list_sessions(self) -> List[Dict]:
        """List all sessions with basic information."""
        session_list = []
        for session_id, session in self.sessions.items():
            session_list.append({
                "session_id": session_id,
                "created_at": session.created_at,
                "last_interaction": session.last_interaction,
                "status": self._get_session_status(session)
            })
        return session_list
    
    def _get_session_status(self, session: AgentSession) -> str:
        """Determine the status of a session."""
        if session.last_interaction is None:
            return "pending"
        
        time_since_last_interaction = datetime.now() - session.last_interaction
        threshold = timedelta(minutes=30)  # From constants
        
        if time_since_last_interaction < threshold:
            return "active"
        else:
            return "inactive"
    
    def cleanup_inactive_sessions(self) -> int:
        """Remove sessions that have been inactive for too long."""
        current_time = datetime.now()
        inactive_threshold = timedelta(hours=24)  # From constants
        
        sessions_to_remove = []
        
        for session_id, session in self.sessions.items():
            if session.last_interaction:
                time_since_last_interaction = current_time - session.last_interaction
                if time_since_last_interaction > inactive_threshold:
                    sessions_to_remove.append(session_id)
        
        for session_id in sessions_to_remove:
            del self.sessions[session_id]
        
        return len(sessions_to_remove)
    
    def update_session_config(self, session_id: str, new_config: AgentConfiguration) -> bool:
        """Update the configuration of an existing session."""
        if session_id not in self.sessions:
            return False
        
        session = self.sessions[session_id]
        session.config = new_config
        self.sessions[session_id] = session
        
        return True
    
    def get_session_config(self, session_id: str) -> Optional[AgentConfiguration]:
        """Get the configuration for a specific session."""
        session = self.get_session(session_id)
        if session:
            return session.config
        return None