"""Grounding validator to ensure agent responses are based on retrieved content."""

from typing import List
from ..models.agent_response import AgentResponse, SourceReference
from ..models.retrieved_content import RetrievedContent
from ..utils.helpers import calculate_confidence_score


class GroundingValidator:
    """Validates that agent responses are properly grounded in retrieved content."""
    
    def __init__(self, grounding_threshold: float = 0.5):
        """Initialize with a grounding threshold."""
        self.grounding_threshold = grounding_threshold
    
    def validate_response_grounding(self, response: AgentResponse, retrieved_content: List[RetrievedContent]) -> bool:
        """Validate that the response is properly grounded in the retrieved content."""
        if not retrieved_content:
            # If no content was retrieved, the response can't be grounded
            return False
        
        if not response.content:
            return False
        
        # Check if response content references information in the retrieved content
        return self._validate_content_alignment(response, retrieved_content)
    
    def _validate_content_alignment(self, response: AgentResponse, retrieved_content: List[RetrievedContent]) -> bool:
        """Internal method to validate alignment between response and retrieved content."""
        response_text = response.content.lower()
        
        # Check if any of the retrieved content appears in the response
        total_source_coverage = 0
        for content_chunk in retrieved_content:
            # Look for phrases/keywords from retrieved content in response
            chunk_text = content_chunk.content.lower()
            
            # Simple overlap check - find common phrases
            chunk_words = set(chunk_text.split())
            response_words = set(response_text.split())
            
            # Calculate intersection
            common_words = chunk_words.intersection(response_words)
            if common_words:
                overlap_ratio = len(common_words) / max(len(chunk_words), len(response_words), 1)
                total_source_coverage += overlap_ratio
        
        # Average coverage across all content chunks
        avg_coverage = total_source_coverage / max(len(retrieved_content), 1)
        
        return avg_coverage >= self.grounding_threshold
    
    def calculate_grounding_score(self, response: AgentResponse, retrieved_content: List[RetrievedContent]) -> float:
        """Calculate a numerical score representing how well the response is grounded."""
        if not retrieved_content or not response.content:
            return 0.0
        
        response_text = response.content.lower()
        
        max_similarity = 0.0
        
        for content_chunk in retrieved_content:
            chunk_text = content_chunk.content.lower()
            
            # Calculate a simple similarity score based on word overlap
            chunk_words = set(chunk_text.split())
            response_words = set(response_text.split())
            
            # Jaccard similarity coefficient
            intersection = len(chunk_words.intersection(response_words))
            union = len(chunk_words.union(response_words))
            
            if union > 0:
                jaccard_sim = intersection / union
                max_similarity = max(max_similarity, jaccard_sim)
        
        return max_similarity
    
    def validate_source_citations(self, response: AgentResponse, retrieved_content: List[RetrievedContent]) -> bool:
        """Validate that the response includes proper citations to the retrieved content."""
        if not retrieved_content or not response.sources:
            return False
        
        # Check if the sources in the response match any of the retrieved content
        matched_sources = 0
        for source in response.sources:
            for content in retrieved_content:
                if self._content_matches_source(content, source):
                    matched_sources += 1
                    break
        
        # Require at least one matching source
        return matched_sources > 0
    
    def _content_matches_source(self, content_chunk: RetrievedContent, source_ref: SourceReference) -> bool:
        """Check if a content chunk matches a source reference."""
        # Compare content IDs if available
        if content_chunk.content_id == source_ref.chunk_index:  # Note: chunk_index might not match ID
            return True
        
        # Compare URLs if available
        if content_chunk.source_url == source_ref.source_url:
            return True
        
        # Compare titles
        if content_chunk.page_title == source_ref.page_title:
            return True
        
        # Compare content previews if available
        if source_ref.content_preview and content_chunk.content.startswith(source_ref.content_preview[:50]):
            return True
        
        return False
    
    def validate_response_quality(self, response: AgentResponse, retrieved_content: List[RetrievedContent]) -> bool:
        """Validate overall response quality including grounding."""
        # Check if this might be a general conversation response (no retrieved content but valid response)
        if not retrieved_content and response.content:
            # For responses without retrieved content, we can't validate grounding
            # So we consider it valid if it has content and is not expected to be grounded
            return True

        grounding_score = self.calculate_grounding_score(response, retrieved_content)

        # Check that the grounding score meets our threshold
        if grounding_score < self.grounding_threshold:
            return False

        # Check that sources are properly cited
        if not self.validate_source_citations(response, retrieved_content):
            return False

        return True