"""Validation utilities for the RAG system."""

from typing import List
from ..models.agent_response import AgentResponse, SourceReference
from ..models.query_request import QueryRequest


def validate_agent_response(response: AgentResponse) -> List[str]:
    """Validate an agent response and return a list of validation errors."""
    errors = []
    
    # Validate content length
    if len(response.content) > 4000:
        errors.append("Response content exceeds maximum character limit of 4000")
    
    # Validate sources list
    if len(response.sources) > 10:
        errors.append("Sources list exceeds maximum limit of 10 items")
    
    # Validate confidence score
    if not 0.0 <= response.confidence <= 1.0:
        errors.append("Confidence score must be between 0.0 and 1.0")
    
    # Validate all sources have valid URLs
    for i, source in enumerate(response.sources):
        if not source.source_url.startswith(("http://", "https://")):
            errors.append(f"Source {i} has invalid URL: {source.source_url}")
    
    return errors


def validate_source_reference(source: SourceReference) -> List[str]:
    """Validate a source reference and return a list of validation errors."""
    errors = []
    
    # Validate URL format
    if not source.source_url.startswith(("http://", "https://")):
        errors.append(f"Invalid URL format: {source.source_url}")
    
    # Validate score ranges
    if not 0.0 <= source.similarity_score <= 1.0:
        errors.append(f"Similarity score must be between 0.0 and 1.0: {source.similarity_score}")
    
    if not 0.0 <= source.relevance_score <= 1.0:
        errors.append(f"Relevance score must be between 0.0 and 1.0: {source.relevance_score}")
    
    # Validate content preview length
    if len(source.content_preview) > 500:
        errors.append(f"Content preview exceeds 500 characters: {len(source.content_preview)}")
    
    return errors


def validate_query_request(query: QueryRequest) -> List[str]:
    """Validate a query request and return a list of validation errors."""
    errors = []
    
    # Validate query text length
    if len(query.query_text) < 1 or len(query.query_text) > 2000:
        errors.append(f"Query text length must be between 1 and 2000 characters: {len(query.query_text)}")
    
    # Validate session ID if provided
    if query.session_id:
        if len(query.session_id) > 50:
            errors.append("Session ID exceeds maximum length of 50 characters")
    
    # Validate user context if provided
    if query.user_context:
        context_size = len(str(query.user_context))
        if context_size > 5 * 1024:  # 5KB limit
            errors.append(f"User context exceeds 5KB limit: {context_size} bytes")
    
    return errors


def validate_response_grounding(response: AgentResponse, original_query: str) -> bool:
    """Validate that the response is grounded in the retrieved content."""
    # Check that the response contains information from the sources
    response_lower = response.content.lower()
    
    # At least one source should be referenced in the response
    has_source_reference = False
    for source in response.sources:
        source_text = source.content_preview.lower()
        if len(source_text.split()) > 1:  # Make sure it's substantial content
            if source_text in response_lower:
                has_source_reference = True
                break
    
    return has_source_reference