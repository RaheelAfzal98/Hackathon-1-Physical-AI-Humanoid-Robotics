"""SourceReference model for referencing original content."""

from pydantic import BaseModel, field_validator


class SourceReference(BaseModel):
    """Reference to original content that grounds an agent response."""
    source_url: str  # URL where the content can be found
    page_title: str  # Title of the page or document
    section_title: str  # Specific section or heading
    chunk_index: int  # Index of the content chunk
    content_preview: str  # Short preview of the referenced content (max 500 chars)
    similarity_score: float  # How similar this content is to the query (0.0-1.0)
    relevance_score: float  # How relevant this content is to the response (0.0-1.0)

    @field_validator('source_url')
    def validate_source_url(cls, v):
        if not v.startswith(('http://', 'https://')):
            raise ValueError('Source URL must be a valid URL')
        return v

    @field_validator('similarity_score', 'relevance_score')
    def validate_score_range(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Scores must be between 0.0 and 1.0')
        return v

    @field_validator('content_preview')
    def validate_content_preview_length(cls, v):
        if len(v) > 500:
            raise ValueError('Content preview must not exceed 500 characters')
        return v