"""Helper functions for the RAG system."""

import time
from datetime import datetime
from typing import Any, Dict, List, Optional
from ..models.agent_response import AgentResponse, SourceReference, ToolCall
from ..models.query_request import QueryRequest


def get_timestamp() -> datetime:
    """Get the current timestamp."""
    return datetime.now()


def calculate_elapsed_time(start_time: float) -> int:
    """Calculate elapsed time in milliseconds since start_time."""
    return int((time.time() - start_time) * 1000)


def merge_dicts(base_dict: Dict[str, Any], update_dict: Dict[str, Any]) -> Dict[str, Any]:
    """Merge two dictionaries, with update_dict taking precedence."""
    merged = base_dict.copy()
    merged.update(update_dict)
    return merged


def sanitize_input(input_text: str) -> str:
    """Sanitize input text by removing potentially harmful content."""
    # Remove any potential dangerous characters or patterns
    sanitized = input_text.replace('<script', '&lt;script').replace('javascript:', 'javascript_')
    return sanitized.strip()


def format_sources_list(sources: List[SourceReference]) -> str:
    """Format a list of sources into a readable string."""
    if not sources:
        return "No sources referenced."
    
    formatted_sources = []
    for i, source in enumerate(sources, 1):
        formatted_sources.append(f"[{i}] {source.page_title} ({source.section_title}): {source.source_url}")
    
    return "\n".join(formatted_sources)


def create_tool_call(tool_name: str, arguments: Dict[str, Any], result: str) -> ToolCall:
    """Create a ToolCall object with current timestamp."""
    return ToolCall(
        tool_name=tool_name,
        arguments=arguments,
        result=result,
        timestamp=get_timestamp()
    )


def calculate_confidence_score(context_relevance: float, source_similarity: float, 
                             response_accuracy: float) -> float:
    """Calculate an overall confidence score based on multiple factors."""
    # Weighted average of different confidence factors
    weights = [0.4, 0.4, 0.2]  # weights for relevance, similarity, and accuracy
    scores = [context_relevance, source_similarity, response_accuracy]
    
    weighted_score = sum(w * s for w, s in zip(weights, scores))
    
    # Clamp the result to [0.0, 1.0]
    return max(0.0, min(1.0, weighted_score))


def extract_entities_from_text(text: str, entity_types: List[str]) -> Dict[str, List[str]]:
    """Extract entities of specified types from text (placeholder implementation)."""
    # This is a simplified implementation - in practice, you'd use NLP libraries
    entities = {}
    text_lower = text.lower()
    
    for entity_type in entity_types:
        # Placeholder implementation - this would use actual NER in production
        entities[entity_type] = []
    
    return entities


def generate_response_id() -> str:
    """Generate a unique response ID."""
    import uuid
    return f"resp_{uuid.uuid4().hex[:12]}"


def calculate_similarity_score(text1: str, text2: str) -> float:
    """Calculate a basic similarity score between two texts."""
    # This is a placeholder implementation - in practice, you'd use more sophisticated methods
    set1 = set(text1.lower().split())
    set2 = set(text2.lower().split())
    
    if not set1 and not set2:
        return 1.0  # Both empty texts are similar
    
    intersection = set1.intersection(set2)
    union = set1.union(set2)
    
    if not union:
        return 0.0
    
    return len(intersection) / len(union)


def normalize_text(text: str) -> str:
    """Normalize text for comparison purposes."""
    import re
    # Remove extra whitespace, convert to lowercase
    normalized = re.sub(r'\s+', ' ', text.strip().lower())
    return normalized