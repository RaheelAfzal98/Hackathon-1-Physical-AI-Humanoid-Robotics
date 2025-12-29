"""Base class for retrieval operations in the RAG system."""

from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from ..models.retrieved_content import RetrievedContent


class RetrievalTool(ABC):
    """Base class for retrieval operations providing the core interface for content retrieval."""
    
    @abstractmethod
    def initialize(self):
        """Initialize the retrieval tool and connect to the vector database."""
        pass

    @abstractmethod
    def retrieve_content(self, query: str, top_k: int = 5, threshold: float = 0.3) -> List[RetrievedContent]:
        """Retrieve relevant content chunks based on the query."""
        pass

    @abstractmethod
    def validate_retrieval(self, query: str, results: List[RetrievedContent]) -> bool:
        """Validate that retrieved results are relevant to the query."""
        pass

    @abstractmethod
    def handle_retrieval_failure(self, query: str, error: Exception) -> List[RetrievedContent]:
        """Handle cases where retrieval fails and provide fallback content."""
        pass

    @abstractmethod
    def update_similarity_threshold(self, threshold: float) -> bool:
        """Update the similarity threshold for content retrieval."""
        pass

    @abstractmethod
    def get_statistics(self) -> Dict[str, Any]:
        """Get statistics about the retrieval system performance."""
        pass