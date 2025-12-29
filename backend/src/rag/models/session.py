"""AgentSession model representing a conversation context."""

from datetime import datetime
from typing import Dict, Optional, Any
from pydantic import BaseModel
from .configuration import AgentConfiguration


class AgentSession(BaseModel):
    """Represents a conversation context including conversation history, user preferences, and temporary state."""
    session_id: str
    thread_id: Optional[str] = None  # Associated OpenAI Assistant thread ID
    created_at: datetime
    last_interaction: Optional[datetime] = None
    config: AgentConfiguration
    metadata: Optional[Dict[str, Any]] = {}