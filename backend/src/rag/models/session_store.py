"""Session store for managing session data."""

from datetime import datetime
from typing import Dict, Optional
from ..models.session import AgentSession
from ..models.configuration import AgentConfiguration


class SessionStore:
    """Store for managing session data in memory."""
    
    def __init__(self):
        """Initialize the session store."""
        self.sessions: Dict[str, AgentSession] = {}
    
    def add_session(self, session: AgentSession) -> bool:
        """Add a session to the store."""
        if session.session_id in self.sessions:
            return False  # Session already exists
        
        self.sessions[session.session_id] = session
        return True
    
    def get_session(self, session_id: str) -> Optional[AgentSession]:
        """Get a session from the store."""
        return self.sessions.get(session_id)
    
    def update_session(self, session: AgentSession) -> bool:
        """Update an existing session in the store."""
        if session.session_id not in self.sessions:
            return False  # Session doesn't exist
        
        self.sessions[session.session_id] = session
        return True
    
    def remove_session(self, session_id: str) -> bool:
        """Remove a session from the store."""
        if session_id in self.sessions:
            del self.sessions[session_id]
            return True
        return False
    
    def get_all_sessions(self) -> Dict[str, AgentSession]:
        """Get all sessions in the store."""
        return self.sessions.copy()
    
    def update_session_config(self, session_id: str, config: AgentConfiguration) -> bool:
        """Update the configuration of a session."""
        session = self.get_session(session_id)
        if not session:
            return False
        
        session.config = config
        return self.update_session(session)
    
    def has_session(self, session_id: str) -> bool:
        """Check if a session exists in the store."""
        return session_id in self.sessions