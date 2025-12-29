"""AgentResponse model containing the structured response."""

from datetime import datetime
from typing import List, Optional
from pydantic import BaseModel


class SourceReference(BaseModel):
    """Reference to original content that grounds an agent response."""
    source_url: str  # URL where the content can be found
    page_title: str  # Title of the page or document
    section_title: str  # Specific section or heading
    chunk_index: int  # Index of the content chunk
    content_preview: str  # Short preview of the referenced content (max 500 chars)
    similarity_score: float  # How similar this content is to the query (0.0-1.0)
    relevance_score: float  # How relevant this content is to the response (0.0-1.0)


class ToolCall(BaseModel):
    """A function call made by the agent during response generation."""
    tool_name: str  # Name of the tool that was called
    arguments: dict  # Arguments passed to the tool
    result: str  # Result returned by the tool
    timestamp: datetime  # When the tool was called


class AgentResponse(BaseModel):
    """Structured response containing the answer text, source references, confidence indicators, and metadata."""
    response_id: str  # Unique identifier for the response
    content: str  # The main response text from the agent (max 4000 characters)
    sources: List[SourceReference]  # List of sources used in the response
    confidence: float  # Confidence level of the response (0.0-1.0)
    tool_calls: List[ToolCall]  # List of tools called during response generation
    created_at: datetime  # Timestamp when response was generated
    query_time_ms: int  # Time taken to generate the response in milliseconds