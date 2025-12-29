import pytest
from unittest.mock import patch, MagicMock
from app.services.vector_storage import create_collection, save_chunk_to_qdrant


class TestVectorStorage:
    """Unit tests for the vector storage service"""
    
    @patch('app.services.vector_storage.client')
    def test_create_collection(self, mock_client):
        """Test create collection function"""
        # Mock the client responses
        mock_collection = MagicMock()
        mock_collection.name = "test_collection"
        mock_client.get_collections.return_value.collections = [mock_collection]
        
        # Test when collection already exists
        result = create_collection("test_collection", vector_size=100)
        assert result is True
        
        # Reset mock to test creating new collection
        mock_client.get_collections.return_value.collections = []
        mock_client.create_collection.return_value = True
        
        result = create_collection("new_collection", vector_size=100)
        assert result is True
        mock_client.create_collection.assert_called_once()
    
    @patch('app.services.vector_storage.client')
    @patch('app.services.vector_storage.generate_chunk_id')
    def test_save_chunk_to_qdrant(self, mock_generate_id, mock_client, ):
        """Test save chunk to Qdrant function"""
        mock_generate_id.return_value = "test-id-123"
        mock_client.upsert.return_value = True
        
        chunk_data = {
            'id': 'test-id-123',
            'text_content': 'Test content',
            'source_url': 'https://example.com',
            'chunk_index': 0,
            'total_chunks': 1,
            'hash': 'abc123',
            'title': 'Test Title'
        }
        vector = [0.1, 0.2, 0.3]
        
        result = save_chunk_to_qdrant(chunk_data, "test_collection", vector)
        
        assert result == "test-id-123"
        mock_client.upsert.assert_called_once()