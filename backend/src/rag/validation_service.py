from typing import Dict, Any, List
from src.rag.client import QdrantClientWrapper
from src.rag.embedding import CohereEmbeddingService
from src.rag.models import ContentChunk


class ValidationService:
    def __init__(self):
        self.qdrant_client = QdrantClientWrapper()
        self.cohere_service = CohereEmbeddingService()

    def validate_source_traceability(self, content_chunks: List[ContentChunk]) -> Dict[str, Any]:
        """
        Validate that the retrieved content chunks can be traced back to original sources.
        
        Args:
            content_chunks: List of ContentChunk objects to validate
            
        Returns:
            Dictionary with validation results
        """
        validation_results = {
            "total_chunks": len(content_chunks),
            "valid_chunks": 0,
            "invalid_chunks": 0,
            "details": []
        }
        
        for i, chunk in enumerate(content_chunks):
            chunk_validation = {
                "chunk_id": chunk.id,
                "valid": True,
                "issues": []
            }
            
            # Check source_url
            if not chunk.metadata.get('source_url'):
                chunk_validation["valid"] = False
                chunk_validation["issues"].append("Missing source_url")
            
            # Check document_id
            if not chunk.metadata.get('document_id'):
                chunk_validation["valid"] = False
                chunk_validation["issues"].append("Missing document_id")
            
            # Check chunk_index
            chunk_index = chunk.metadata.get('chunk_index')
            if chunk_index is None or (isinstance(chunk_index, int) and chunk_index < 0):
                chunk_validation["valid"] = False
                chunk_validation["issues"].append("Missing or invalid chunk_index")
            
            # Check page_title
            if not chunk.metadata.get('page_title'):
                chunk_validation["valid"] = False
                chunk_validation["issues"].append("Missing page_title")
            
            if chunk_validation["valid"]:
                validation_results["valid_chunks"] += 1
            else:
                validation_results["invalid_chunks"] += 1
                
            validation_results["details"].append(chunk_validation)
        
        validation_results["traceability_score"] = (
            validation_results["valid_chunks"] / validation_results["total_chunks"] 
            if validation_results["total_chunks"] > 0 
            else 0
        )
        
        return validation_results

    def validate_content_integrity(self, original_content: str, retrieved_content: str) -> Dict[str, Any]:
        """
        Validate that the retrieved content matches the original content.
        
        Args:
            original_content: The original content
            retrieved_content: The content retrieved from the system
            
        Returns:
            Dictionary with validation results
        """
        # Check basic similarity using a simple overlap ratio
        original_words = set(original_content.lower().split())
        retrieved_words = set(retrieved_content.lower().split())
        
        if len(original_words) == 0:
            similarity = 1.0 if len(retrieved_words) == 0 else 0.0
        else:
            overlap = len(original_words.intersection(retrieved_words))
            similarity = overlap / len(original_words)
        
        return {
            "similarity_ratio": similarity,
            "original_length": len(original_content),
            "retrieved_length": len(retrieved_content),
            "is_intact": similarity > 0.8  # Threshold for content integrity
        }

    def validate_pipeline_connectivity(self) -> Dict[str, Any]:
        """
        Validate connectivity to all required services.
        
        Returns:
            Dictionary with connectivity validation results
        """
        results = {
            "qdrant": {"connected": False, "details": ""},
            "cohere": {"connected": False, "details": ""}
        }
        
        # Test Qdrant connectivity
        try:
            qdrant_connected = self.qdrant_client.check_connection()
            results["qdrant"]["connected"] = qdrant_connected
            results["qdrant"]["details"] = "Successfully connected to Qdrant" if qdrant_connected else "Failed to connect to Qdrant"
        except Exception as e:
            results["qdrant"]["connected"] = False
            results["qdrant"]["details"] = f"Qdrant connection error: {str(e)}"
        
        # Test Cohere connectivity (just test if client is configured properly)
        try:
            if self.cohere_service.client:
                results["cohere"]["connected"] = True
                results["cohere"]["details"] = "Cohere client is configured"
            else:
                results["cohere"]["connected"] = False
                results["cohere"]["details"] = "Cohere client not properly configured"
        except Exception as e:
            results["cohere"]["connected"] = False
            results["cohere"]["details"] = f"Cohere connection error: {str(e)}"
        
        return results

    def validate_embedding_consistency(self, text: str) -> Dict[str, Any]:
        """
        Validate that embeddings are generated consistently.
        
        Args:
            text: Text to generate embeddings for
            
        Returns:
            Dictionary with consistency validation results
        """
        try:
            # Generate embedding multiple times to check consistency
            embedding1 = self.cohere_service.get_embedding(text)
            embedding2 = self.cohere_service.get_embedding(text)
            
            # Check if embeddings are identical (they should be for the same input)
            is_consistent = embedding1 == embedding2
            embedding_dimension = len(embedding1) if embedding1 else 0
            
            return {
                "consistent": is_consistent,
                "dimension": embedding_dimension,
                "details": "Embeddings are consistent" if is_consistent else "Embeddings are inconsistent"
            }
        except Exception as e:
            return {
                "consistent": False,
                "dimension": 0,
                "details": f"Error validating embedding consistency: {str(e)}"
            }