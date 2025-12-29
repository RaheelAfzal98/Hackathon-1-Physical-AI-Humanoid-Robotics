"""Service to manage integration with the existing retrieval pipeline."""

from typing import List
from ..models.retrieved_content import RetrievedContent
from .qdrant_adapter import QdrantAdapter
from ..config.agent_config import AgentConfig
from ..utils.validation import validate_source_reference
from ..models.source_reference import SourceReference


class RetrievalIntegrationService:
    """Service to handle integration with the existing retrieval pipeline."""
    
    def __init__(self, qdrant_adapter: QdrantAdapter):
        """Initialize the retrieval integration service."""
        self.qdrant_adapter = qdrant_adapter
    
    def fetch_relevant_content(self, query: str, top_k: int = 5, threshold: float = 0.3) -> List[RetrievedContent]:
        """Fetch relevant content from the existing retrieval pipeline."""
        try:
            return self.qdrant_adapter.retrieve_content(
                query=query,
                top_k=top_k,
                threshold=threshold
            )
        except Exception as e:
            self.qdrant_adapter.logger.error(f"Error in fetch_relevant_content: {str(e)}")
            # Handle the error by calling the fallback mechanism
            return self.qdrant_adapter.handle_retrieval_failure(query, e)
    
    def validate_content_quality(self, query: str, retrieved_content: List[RetrievedContent]) -> bool:
        """Validate the quality and relevance of retrieved content."""
        return self.qdrant_adapter.validate_retrieval(query, retrieved_content)
    
    def generate_source_references(self, retrieved_content: List[RetrievedContent]) -> List[SourceReference]:
        """Generate source references from retrieved content for agent responses."""
        source_refs = []
        
        for content in retrieved_content:
            source_ref = SourceReference(
                source_url=content.source_url,
                page_title=content.page_title,
                section_title="",  # Will need to extract from content if available
                chunk_index=content.metadata.get('chunk_index', 0) if isinstance(content.metadata, dict) else 0,
                content_preview=content.content[:200] + "..." if len(content.content) > 200 else content.content,
                similarity_score=content.similarity_score,
                relevance_score=content.similarity_score  # For now, using similarity as relevance
            )
            source_refs.append(source_ref)
        
        return source_refs
    
    def update_retrieval_parameters(self, top_k: int = None, threshold: float = None):
        """Update retrieval parameters like top_k and similarity threshold."""
        if threshold is not None:
            self.qdrant_adapter.update_similarity_threshold(threshold)
        
        # For top_k, this is typically passed per query, but we could store defaults
        if top_k is not None:
            # Store this as a default for future queries if needed
            pass
    
    def get_retrieval_stats(self) -> dict:
        """Get statistics about the retrieval system."""
        return self.qdrant_adapter.get_statistics()
    
    def handle_retrieval_error(self, query: str, error: Exception) -> List[RetrievedContent]:
        """Handle errors in the retrieval process."""
        return self.qdrant_adapter.handle_retrieval_failure(query, error)