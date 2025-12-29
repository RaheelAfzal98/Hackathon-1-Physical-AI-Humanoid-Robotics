"""RetrievedContent model for content chunks from Qdrant."""

from typing import Dict, Optional
from pydantic import BaseModel, validator


class RetrievedContent(BaseModel):
    """Content chunks retrieved from Qdrant with their metadata that ground agent responses."""
    content_id: str  # Identifier for the content chunk
    content: str  # The actual content text
    metadata: Dict  # Metadata associated with the content
    source_url: str  # URL of the original document
    page_title: str  # Title of the document
    similarity_score: float  # Similarity score from the query
    vector_id: str  # ID in the vector database
    token_count: int  # Number of tokens in the content

    @validator('similarity_score')
    def similarity_score_range(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('similarity_score must be between 0.0 and 1.0')
        return v

    @validator('content')
    def content_length(cls, v):
        if len(v) > 4000:
            raise ValueError('content must not exceed 4000 characters')
        return v