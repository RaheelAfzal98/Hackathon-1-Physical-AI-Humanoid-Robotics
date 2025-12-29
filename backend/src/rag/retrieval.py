from typing import List, Dict, Any, Optional
import time
import logging
from src.rag.client import QdrantClientWrapper
from src.rag.embedding import get_embedding_for_text, process_query_embedding
from src.rag.models import (
    QueryRequest,
    RankedChunk,
    ContentChunk,
    RetrievalParameters,
    RetrievalResponse
)

logger = logging.getLogger(__name__)


class RetrievalService:
    def __init__(self):
        self.qdrant_client = QdrantClientWrapper()

    def retrieve(
        self, 
        query_request: QueryRequest
    ) -> RetrievalResponse:
        """
        Perform the full retrieval process: query embedding, vector search, result formatting.
        
        Args:
            query_request: The query request containing text and parameters
            
        Returns:
            RetrievalResponse with ranked results
        """
        start_time = time.time()
        
        # Step 0: Validate the query request parameters
        if query_request.top_k < 1 or query_request.top_k > 20:
            raise ValueError(f"top_k must be between 1-20, got {query_request.top_k}")

        if query_request.similarity_threshold < 0.0 or query_request.similarity_threshold > 1.0:
            raise ValueError(f"similarity_threshold must be between 0.0-1.0, got {query_request.similarity_threshold}")

        # Step 1: Generate embedding for the query text
        query_vector = process_query_embedding(query_request.text)
        
        # Step 2: Perform vector similarity search in Qdrant
        search_results = self.qdrant_client.search(
            query_vector=query_vector,
            top_k=query_request.top_k,
            similarity_threshold=query_request.similarity_threshold,
            filters=query_request.filters
        )
        
        # Step 3: Format results into RankedChunk objects
        ranked_chunks = []
        for idx, result in enumerate(search_results):
            # Create a ContentChunk from the search result
            content_chunk = ContentChunk(
                id=result["id"],
                content=result["payload"]["content"],
                metadata=result["payload"]["metadata"],
                embedding=None  # Don't return the full embedding to save bandwidth
            )
            
            # Create a RankedChunk with the calculated rank and similarity score
            ranked_chunk = RankedChunk(
                chunk=content_chunk,
                similarity_score=result["similarity_score"],
                rank=idx
            )
            
            ranked_chunks.append(ranked_chunk)
        
        # Calculate total time for the operation
        search_time_ms = (time.time() - start_time) * 1000
        
        # Create RetrievalParameters object for the response
        retrieval_params = RetrievalParameters(
            top_k=query_request.top_k,
            similarity_threshold=query_request.similarity_threshold,
            collection_name=self.qdrant_client.collection_name,
            filters=query_request.filters
        )
        
        # Create and return the final response
        response = RetrievalResponse(
            query_text=query_request.text,
            results=ranked_chunks,
            total_chunks_processed=len(search_results),  # Note: This is actually the number of returned chunks
            search_time_ms=search_time_ms,
            retrieval_parameters=retrieval_params
        )
        
        return response

    def validate_connection(self) -> bool:
        """Validate connection to Qdrant."""
        return self.qdrant_client.check_connection()