import pytest
from unittest.mock import patch, MagicMock
from app.services.embedding_service import embed


class TestEmbeddingService:
    """Unit tests for the embedding service"""
    
    @patch('app.services.embedding_service.co')
    def test_embed_function(self, mock_cohere):
        """Test embedding function with mocked Cohere client"""
        # Mock the Cohere embed response
        mock_response = MagicMock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        mock_cohere.embed.return_value = mock_response
        
        text_chunks = ["First chunk", "Second chunk"]
        result = embed(text_chunks, model='embed-multilingual-v3.0')
        
        # Verify that Cohere's embed function was called
        mock_cohere.embed.assert_called_once()
        
        # Verify that the result is a list of embeddings
        assert len(result) == 2  # Should have embeddings for both chunks
        assert len(result[0]) == 3  # Each embedding should have 3 dimensions in mock