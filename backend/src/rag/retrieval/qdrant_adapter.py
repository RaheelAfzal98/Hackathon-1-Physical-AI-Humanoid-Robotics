"""Adapter for the existing Qdrant retrieval pipeline."""

from typing import List, Dict, Any, Optional
import logging
from ..models.retrieved_content import RetrievedContent
from ..retrieval.retrieval_tool import RetrievalTool
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Filter, FieldCondition, MatchValue
from ..config.agent_config import AgentConfig
from ..utils.helpers import get_timestamp
from ...config.settings import settings


class QdrantAdapter(RetrievalTool):
    """Adapter class to integrate with the existing Qdrant-based retrieval pipeline."""

    def __init__(self, config: AgentConfig):
        """Initialize the Qdrant adapter."""
        self.config = config
        self.client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            https=True if "https" in config.qdrant_url else False
        )
        self.collection_name = config.qdrant_collection_name
        self.logger = logging.getLogger(__name__)

        # Initialize embedding service - only Cohere for now
        self.use_cohere = bool(config.cohere_api_key and config.cohere_api_key != "your_cohere_api_key_here")

        if self.use_cohere:
            import cohere
            self.cohere_client = cohere.Client(config.cohere_api_key)
        else:
            raise ValueError("No valid embedding service configured. Please set COHERE_API_KEY.")

    def initialize(self):
        """Initialize the retrieval tool and verify connection to Qdrant."""
        try:
            # Check if collection exists
            collection_info = self.client.get_collection(self.collection_name)
            self.logger.info(f"Connected to Qdrant collection '{self.collection_name}' with {collection_info.points_count} points")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to Qdrant collection '{self.collection_name}': {str(e)}")
            raise

    def retrieve_content(self, query: str, top_k: int = 5, threshold: float = 0.3) -> List[RetrievedContent]:
        """Retrieve relevant content chunks from Qdrant based on the query."""
        try:
            # Generate embedding for the query using Cohere
            if self.use_cohere:
                query_embedding_response = self.cohere_client.embed(
                    texts=[query],
                    model="embed-english-v3.0",  # Using the same model as specified in requirements
                    input_type="search_query"
                )
                query_embedding = query_embedding_response.embeddings[0]
            else:
                raise ValueError("No valid embedding service available")

            # Perform semantic search in Qdrant using the correct method for newer Qdrant versions
            try:
                # Try the newer query_points method first (Qdrant 1.9.0+)
                search_results = self.client.query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=top_k,
                    score_threshold=threshold,
                    with_payload=True,
                    with_vectors=False
                )
                # Handle the response format for newer API
                if hasattr(search_results, 'points'):
                    search_results = search_results.points
            except AttributeError:
                # Fallback to the older search method if query_points doesn't exist
                search_results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=top_k,
                    score_threshold=threshold,
                    with_payload=True,
                    with_vectors=False
                )

            # Convert results to RetrievedContent objects
            retrieved_content = []
            for result in search_results:
                payload = result.payload
                content = payload.get("content", "")
                metadata = payload.get("metadata", {})

                # Extract specific metadata fields if they exist
                source_url = metadata.get("source_url", payload.get("source_url", ""))
                page_title = metadata.get("page_title", payload.get("page_title", "Unknown"))

                # Create RetrievedContent object
                retrieved_chunk = RetrievedContent(
                    content_id=result.id,
                    content=content,
                    metadata=metadata,
                    source_url=source_url,
                    page_title=page_title,
                    similarity_score=result.score,
                    vector_id=result.id,  # Using the result ID as the vector ID
                    token_count=len(content.split())  # Approximate token count
                )

                retrieved_content.append(retrieved_chunk)

            return retrieved_content

        except Exception as e:
            self.logger.error(f"Error retrieving content from Qdrant: {str(e)}")
            raise

    def validate_retrieval(self, query: str, results: List[RetrievedContent]) -> bool:
        """Validate that retrieved results are relevant to the query."""
        # Basic validation: check if we got results and they have content
        if not results:
            return False

        # Check if the average similarity score is above a minimum threshold
        avg_score = sum(r.similarity_score for r in results) / len(results)
        min_acceptable_score = getattr(self.config, 'grounding_threshold', 0.1)  # Default to 0.1

        return avg_score >= min_acceptable_score

    def handle_retrieval_failure(self, query: str, error: Exception) -> List[RetrievedContent]:
        """Handle cases where retrieval fails and provide fallback content."""
        self.logger.warning(f"Retrieval failed for query '{query[:50]}...': {str(error)}")

        # In a fallback scenario, we might return empty results or
        # use a different retrieval strategy
        if self.config.fallback_enabled:
            # For now, return empty list - in a real implementation,
            # you might try different strategies like keyword search
            return []
        else:
            # If fallbacks are disabled, re-raise the error
            raise error

    def update_similarity_threshold(self, threshold: float) -> bool:
        """Update the similarity threshold for content retrieval."""
        # This could be implemented to adjust search behavior
        # For now, we'll just log the change
        self.logger.info(f"Similarity threshold updated to {threshold}")
        return True

    def get_statistics(self) -> dict:
        """Get statistics about the retrieval system performance."""
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "total_points": collection_info.points_count,
                "vector_size": collection_info.config.params.vectors.size,
                "distance_type": collection_info.config.params.vectors.distance
            }
        except Exception as e:
            self.logger.error(f"Error getting Qdrant statistics: {str(e)}")
            return {"error": str(e)}