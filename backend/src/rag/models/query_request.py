"""QueryRequest model containing user input with session context."""

from typing import Dict, Optional, Any
from pydantic import BaseModel, field_validator


class QueryRequest(BaseModel):
    """User input containing the question and optional session context."""
    query_text: str  # The actual question or query text
    session_id: Optional[str] = None  # Optional session ID for stateful interactions
    user_context: Optional[Dict[str, Any]] = None  # Additional context about the user or situation
    response_options: Optional[Dict[str, Any]] = None  # Specific options for this query (overrides session config)

    @field_validator('query_text')
    def validate_query_text(cls, v):
        if not v or len(v) < 1:
            raise ValueError('Query text must not be empty')
        if len(v) > 2000:
            raise ValueError('Query text must be less than 2000 characters')
        return v

    @field_validator('user_context')
    def validate_user_context(cls, v):
        if v and len(str(v)) > 5 * 1024:  # 5KB limit
            raise ValueError('User context must not exceed 5KB')
        return v