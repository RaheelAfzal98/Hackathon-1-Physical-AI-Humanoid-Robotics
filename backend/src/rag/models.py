from pydantic import BaseModel
from pydantic.functional_validators import field_validator
from typing import List, Dict, Any, Optional
from datetime import datetime


class QueryRequest(BaseModel):
    text: str
    top_k: int = 5
    similarity_threshold: float = 0.5
    filters: Optional[Dict[str, Any]] = None

    @field_validator('text')
    @classmethod
    def validate_text(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('Query text must not be empty')
        if len(v) > 1000:
            raise ValueError('Query text must be between 1-1000 characters')
        return v

    @field_validator('top_k')
    @classmethod
    def validate_top_k(cls, v):
        if v < 1 or v > 20:
            raise ValueError('top_k must be between 1-20')
        return v

    @field_validator('similarity_threshold')
    @classmethod
    def validate_similarity_threshold(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('similarity_threshold must be between 0.0-1.0')
        return v


class ContentChunk(BaseModel):
    id: str
    content: str
    embedding: Optional[List[float]] = None  # Embedding might not always be returned
    metadata: Dict[str, Any]
    created_at: Optional[datetime] = None

    @field_validator('id')
    @classmethod
    def validate_id(cls, v):
        if not v:
            raise ValueError('ID must not be empty')
        return v

    @field_validator('content')
    @classmethod
    def validate_content(cls, v):
        if not v:
            raise ValueError('Content must not be empty')
        if len(v) > 10000:
            raise ValueError('Content must not exceed 10,000 characters')
        return v

    @field_validator('metadata')
    @classmethod
    def validate_metadata(cls, v):
        required_fields = ['source_url', 'page_title', 'document_id']
        for field in required_fields:
            if field not in v:
                raise ValueError(f'Metadata must contain required field: {field}')

        # Validate chunk_index if present
        chunk_index = v.get('chunk_index')
        if chunk_index is not None and (not isinstance(chunk_index, int) or chunk_index < 0):
            raise ValueError('chunk_index must be a non-negative integer if present')

        return v


class RankedChunk(BaseModel):
    chunk: ContentChunk
    similarity_score: float
    rank: int

    @field_validator('similarity_score')
    @classmethod
    def validate_similarity_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Similarity score must be between 0.0-1.0')
        return v

    @field_validator('rank')
    @classmethod
    def validate_rank(cls, v):
        if v < 0:
            raise ValueError('Rank must be non-negative')
        return v


class RetrievalParameters(BaseModel):
    top_k: int
    similarity_threshold: float
    collection_name: str
    filters: Optional[Dict[str, Any]] = None


class RetrievalResponse(BaseModel):
    query_text: str
    results: List[RankedChunk]
    total_chunks_processed: int
    search_time_ms: float
    retrieval_parameters: RetrievalParameters


class ValidationError(BaseModel):
    error_code: str
    message: str
    details: Optional[Dict[str, Any]] = None
    timestamp: datetime

    @field_validator('timestamp', mode='before')
    @classmethod
    def validate_timestamp(cls, v):
        if v is None:
            return datetime.now()
        return v


class PipelineValidationResponse(BaseModel):
    status: str  # "success" or "failure"
    message: str
    results: List[Dict[str, Any]]  # List of validation test results