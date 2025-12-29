from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from typing import List, Dict, Any, Optional
import logging
from src.config.settings import settings


class QdrantClientWrapper:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            port=settings.qdrant_port,
            # Enable HTTPS if needed
            https=True if "https" in settings.qdrant_url else False
        )
        self.collection_name = settings.qdrant_collection_name
        self.logger = logging.getLogger(__name__)

    def check_connection(self) -> bool:
        """Check if we can connect to the Qdrant instance."""
        try:
            # Try to get collection info to verify connection
            self.client.get_collection(collection_name=self.collection_name)
            self.logger.info(f"Successfully connected to Qdrant collection: {self.collection_name}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to Qdrant: {str(e)}")
            return False

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        similarity_threshold: float = 0.5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform vector similarity search in Qdrant.
        
        Args:
            query_vector: The vector representation of the query
            top_k: Number of results to retrieve
            similarity_threshold: Minimum similarity score for inclusion
            filters: Optional filters for metadata-based query refinement
            
        Returns:
            List of dictionaries containing the search results with payload and scores
        """
        try:
            # Prepare filters if provided
            search_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )
                
                if filter_conditions:
                    search_filters = models.Filter(
                        must=filter_conditions
                    )

            # Perform the search using the newer query_points method
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                score_threshold=similarity_threshold,
                with_payload=True,
                with_vectors=False,  # We don't need the vectors back
                query_filter=search_filters
            )

            # Process and return the results
            # In the new API, the results object might be different
            formatted_results = []

            # Check if results is a ScoredPoint object or a list of them
            if hasattr(results, 'points'):
                # It's a QueryResponse object
                search_results = results.points
            else:
                # It's a list of ScoredPoint objects
                search_results = results

            for result in search_results:
                # Ensure complete metadata is preserved
                payload = result.payload if result.payload is not None else {}
                if 'metadata' not in payload:
                    payload['metadata'] = {}

                # Verify required metadata fields exist
                required_metadata = ['source_url', 'page_title', 'document_id']
                for field in required_metadata:
                    if field not in payload['metadata']:
                        payload['metadata'][field] = f"missing_{field}"

                formatted_results.append({
                    "id": result.id,
                    "payload": payload,
                    "similarity_score": result.score
                })

            return formatted_results

        except Exception as e:
            self.logger.error(f"Error during Qdrant search: {str(e)}")
            raise e

    def get_point(self, point_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific point by its ID.
        
        Args:
            point_id: The ID of the point to retrieve
            
        Returns:
            The point data or None if not found
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id],
                with_payload=True,
                with_vectors=False
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "payload": record.payload
                }

            return None
        except Exception as e:
            self.logger.error(f"Error retrieving point {point_id}: {str(e)}")
            return None