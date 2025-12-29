"""ToolCall model for function calls made by agents."""

from datetime import datetime
from typing import Dict
from pydantic import BaseModel


class ToolCall(BaseModel):
    """A function call made by the agent during response generation."""
    tool_name: str  # Name of the tool that was called
    arguments: Dict  # Arguments passed to the tool
    result: str  # Result returned by the tool
    timestamp: datetime  # When the tool was called