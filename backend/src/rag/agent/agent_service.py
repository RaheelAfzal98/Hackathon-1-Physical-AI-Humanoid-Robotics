"""Base class for agent operations in the RAG system."""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
from ..models.query_request import QueryRequest
from ..models.agent_response import AgentResponse


class AgentService(ABC):
    """Base class for agent operations providing the core interface for agent interactions."""
    
    @abstractmethod
    def create_agent(self, config: Optional[Dict[str, Any]] = None):
        """Initialize a new agent instance with the given configuration."""
        pass

    @abstractmethod
    def process_query(self, query_request: QueryRequest) -> AgentResponse:
        """Process a query and return a response from the agent."""
        pass

    @abstractmethod
    def update_configuration(self, config_updates: Dict[str, Any]) -> bool:
        """Update the agent's configuration parameters."""
        pass

    @abstractmethod
    def get_configuration(self) -> Dict[str, Any]:
        """Get the current agent configuration."""
        pass

    @abstractmethod
    def create_session(self, session_config: Optional[Dict[str, Any]] = None) -> str:
        """Create a new agent session and return the session ID."""
        pass

    @abstractmethod
    def close_session(self, session_id: str) -> bool:
        """Close an existing agent session."""
        pass