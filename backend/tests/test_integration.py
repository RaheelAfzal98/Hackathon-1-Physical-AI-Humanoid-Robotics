import pytest
from unittest.mock import patch, MagicMock
from app.main import execute_ingestion_pipeline


class TestIntegration:
    """Integration test for the complete pipeline"""
    
    @patch('app.main.get_all_urls')
    @patch('app.main.create_collection')
    @patch('app.main.extract_text_from_url')
    @patch('app.main.chunk_text')
    @patch('app.main.embed')
    @patch('app.main.save_chunk_to_qdrant')
    def test_complete_pipeline_integration(self, mock_save_chunk, mock_embed, mock_chunk_text, 
                                           mock_extract_text, mock_create_collection, mock_get_all_urls):
        """Test that all components work together correctly"""
        # Set up mocks
        mock_get_all_urls.return_value = [
            "https://example.com/page1", 
            "https://example.com/page2"
        ]
        mock_create_collection.return_value = True
        mock_extract_text.return_value = {
            'text': 'Sample content for testing',
            'title': 'Test Page',
            'hash': 'abc123',
            'created_at': '2023-01-01T00:00:00',
            'updated_at': '2023-01-01T00:00:00'
        }
        mock_chunk_text.return_value = [{
            'text': 'Sample content for testing',
            'chunk_index': 0,
            'total_chunks': 1
        }]
        mock_embed.return_value = [[0.1, 0.2, 0.3]]
        mock_save_chunk.return_value = "test-id-123"
        
        # Execute the pipeline
        result = execute_ingestion_pipeline(
            base_url="https://example.com",
            collection_name="test_collection"
        )
        
        # Verify results
        assert result['urls_processed'] == 2  # Both URLs processed
        assert result['total_urls_found'] == 2  # Both URLs found
        assert result['chunks_created'] == 2  # Each URL created 1 chunk
        assert result['collection_name'] == "test_collection"
        
        # Verify all services were called appropriately
        mock_get_all_urls.assert_called_once()
        mock_create_collection.assert_called_once()
        assert mock_extract_text.call_count == 2  # Called for each URL
        assert mock_chunk_text.call_count == 2  # Called for each URL
        assert mock_embed.call_count == 2  # Called for each URL
        assert mock_save_chunk.call_count == 2  # Called for each chunk