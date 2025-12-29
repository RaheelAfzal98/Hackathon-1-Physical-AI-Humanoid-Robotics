"""Agent Configuration model for managing agent behavior parameters."""

from pydantic import BaseModel, field_validator
from typing import Literal, Optional


class AgentConfiguration(BaseModel):
    """Parameters that control agent behavior including response style, grounding strictness, and tool usage preferences."""
    temperature: float = 0.7  # Controls randomness in agent responses (0.0-1.0)
    grounding_strictness: float = 0.5  # Threshold for how strictly to ground responses (0.0-1.0)
    retrieval_top_k: int = 5  # Number of results to retrieve from the vector database (1-20)
    similarity_threshold: float = 0.3  # Minimum similarity score for retrieved results (0.0-1.0)
    response_format: Literal['standard', 'detailed', 'concise'] = 'standard'  # Desired response format
    fallback_enabled: bool = True  # Whether to use fallback responses when retrieval fails
    enable_citations: bool = True  # Whether to include citations in responses

    @field_validator('temperature')
    def validate_temperature(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Temperature must be between 0.0 and 1.0')
        return v

    @field_validator('grounding_strictness')
    def validate_grounding_strictness(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Grounding strictness must be between 0.0 and 1.0')
        return v

    @field_validator('retrieval_top_k')
    def validate_retrieval_top_k(cls, v):
        if v < 1 or v > 20:
            raise ValueError('retrieval_top_k must be between 1 and 20')
        return v

    @field_validator('similarity_threshold')
    def validate_similarity_threshold(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Similarity threshold must be between 0.0 and 1.0')
        return v


# Create an alias for backward compatibility
AgentConfig = AgentConfiguration