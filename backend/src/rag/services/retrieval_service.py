"""Service for handling retrieval operations in the RAG system."""

from typing import List, Dict, Any
import logging
from ..models.query_request import QueryRequest
from ..models.agent_response import AgentResponse
from ..retrieval.qdrant_adapter import QdrantAdapter
from ..config.agent_config import AgentConfig
from ...config.settings import settings  # Import the main settings


class RetrievalService:
    """Service to handle content retrieval operations for the API."""

    def __init__(self):
        """Initialize the retrieval service."""
        # Use the main settings for Qdrant configuration
        # Create a temporary config object with the required attributes
        class Config:
            def __init__(self):
                self.qdrant_url = settings.qdrant_url
                self.qdrant_api_key = settings.qdrant_api_key
                self.qdrant_collection_name = settings.qdrant_collection_name
                self.cohere_api_key = settings.cohere_api_key
                self.fallback_enabled = True  # Default fallback setting

        self.config = Config()
        self.qdrant_adapter = QdrantAdapter(self.config)
        self.logger = logging.getLogger(__name__)

        # Initialize the adapter
        try:
            self.qdrant_adapter.initialize()
            self.initialized = True
        except Exception as e:
            self.logger.warning(f"Qdrant connection failed (this is expected if no valid credentials are provided): {str(e)}")
            self.initialized = False

    def retrieve(self, query_request: QueryRequest):
        """Retrieve relevant content based on the query request."""
        try:
            if not self.initialized:
                # If Qdrant is not initialized, return an appropriate response
                self.logger.warning("Qdrant not initialized, returning empty results")
                return AgentResponse(
                    response_id="temp_id",
                    content="The RAG system is not properly configured. Please check your API keys.",
                    sources=[],
                    query_text=query_request.query_text,
                    timestamp="2023-01-01T00:00:00Z",
                    query_time_ms=0,
                    session_id=query_request.session_id,
                    grounding_score=0.0,
                    metadata={}
                )

            # Extract parameters from the query request
            query_text = query_request.query_text
            top_k = getattr(query_request, 'top_k', 5)
            similarity_threshold = getattr(query_request, 'similarity_threshold', 0.3)

            # Retrieve content using the Qdrant adapter
            retrieved_content = self.qdrant_adapter.retrieve_content(
                query=query_text,
                top_k=top_k,
                threshold=similarity_threshold
            )

            # Convert retrieved content to the expected response format
            from datetime import datetime
            from uuid import uuid4
            # Import the models needed for the response
            from ..models.agent_response import SourceReference, ToolCall

            # Process the retrieved content to create proper response
            content_text = ""
            sources_list = []

            if retrieved_content:
                # Combine the content from retrieved chunks
                content_text = "\n\n".join([chunk.content for chunk in retrieved_content if chunk.content])

                # Create source references from retrieved content
                for idx, chunk in enumerate(retrieved_content):
                    source_ref = SourceReference(
                        source_url=chunk.source_url,
                        page_title=chunk.page_title,
                        section_title=getattr(chunk, 'section_title', 'General'),  # Use 'General' if section_title not available
                        chunk_index=idx,
                        content_preview=chunk.content[:500] + "..." if len(chunk.content) > 500 else chunk.content,
                        similarity_score=chunk.similarity_score,
                        relevance_score=chunk.similarity_score
                    )
                    sources_list.append(source_ref)
            else:
                content_text = "No relevant content found for the given query."

            # Calculate a confidence score based on average similarity
            if retrieved_content:
                avg_similarity = sum([chunk.similarity_score for chunk in retrieved_content]) / len(retrieved_content)
                confidence = min(1.0, max(0.0, avg_similarity))  # Ensure it's between 0 and 1
            else:
                confidence = 0.0

            # Create the response object with all required fields
            response = AgentResponse(
                response_id=str(uuid4()),
                content=content_text,
                sources=sources_list,
                confidence=confidence,
                tool_calls=[],  # No tool calls in this basic implementation
                created_at=datetime.utcnow(),
                query_time_ms=0,  # This would be calculated in a full implementation
                query_text=query_text,
                session_id=query_request.session_id,
                grounding_score=confidence,  # Using confidence as grounding score
                metadata={"retrieved_chunks_count": len(retrieved_content) if retrieved_content else 0}
            )

            return response

        except Exception as e:
            self.logger.error(f"Error in retrieve: {str(e)}")
            raise

    def validate_connection(self) -> bool:
        """Validate the connection to the retrieval system."""
        if not self.initialized:
            return False

        try:
            # Try to get collection info to verify connection
            stats = self.qdrant_adapter.get_statistics()
            return "error" not in stats
        except Exception:
            return False