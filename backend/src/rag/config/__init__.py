"""Config package for the RAG system."""

from .agent_config import AgentConfiguration

# Create an alias for backward compatibility
AgentConfig = AgentConfiguration

__all__ = ["AgentConfiguration", "AgentConfig"]