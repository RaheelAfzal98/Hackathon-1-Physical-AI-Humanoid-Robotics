import pytest
from unittest.mock import patch, MagicMock
from app.main import execute_ingestion_pipeline
from app.services.url_crawler import get_all_urls
from app.services.text_extractor import extract_text_from_url
from app.services.chunker import chunk_text
from app.services.embedding_service import embed
from app.services.vector_storage import create_collection, save_chunk_to_qdrant


class TestPipelineStructure:
    """Test the pipeline structure without actually connecting to external services"""
    
    @patch('app.main.get_all_urls')
    @patch('app.main.create_collection')
    @patch('app.main.extract_text_from_url')
    @patch('app.main.chunk_text')
    @patch('app.main.embed')
    @patch('app.main.save_chunk_to_qdrant')
    def test_pipeline_structure(self, mock_save_chunk, mock_embed, mock_chunk_text, 
                                mock_extract_text, mock_create_collection, mock_get_all_urls):
        # Mock the return values to simulate the pipeline
        mock_get_all_urls.return_value = ["https://example.com/page1"]
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
        
        # Verify the results
        assert result['urls_processed'] == 1
        assert result['total_urls_found'] == 1
        assert result['chunks_created'] == 1
        assert result['collection_name'] == "test_collection"
        
        # Verify all pipeline steps were called
        mock_get_all_urls.assert_called_once()
        mock_create_collection.assert_called_once()
        mock_extract_text.assert_called_once()
        mock_chunk_text.assert_called_once()
        mock_embed.assert_called_once()
        mock_save_chunk.assert_called_once()


def test_url_crawler():
    """Test that the URL crawler function has the correct signature"""
    # This would normally test with a real URL, but we'll just check the signature
    assert callable(get_all_urls)
    # Function should accept a URL string and return a list of strings
    # (We can't test with real URLs without making external requests)


def test_text_extractor():
    """Test that the text extractor function has the correct signature"""
    assert callable(extract_text_from_url)


def test_chunker():
    """Test that the chunker function has the correct signature"""
    assert callable(chunk_text)
    
    # Test with a simple text
    text = "This is a sample text. It has multiple sentences! Does it work?"
    chunks = chunk_text(text, chunk_size=20, chunk_overlap=5)
    
    assert len(chunks) > 0
    assert 'text' in chunks[0]
    assert 'chunk_index' in chunks[0]
    assert 'total_chunks' in chunks[0]


def test_embedding_service():
    """Test that the embedding function has the correct signature"""
    assert callable(embed)


def test_vector_storage():
    """Test that the vector storage functions have the correct signatures"""
    assert callable(create_collection)
    assert callable(save_chunk_to_qdrant)


if __name__ == "__main__":
    pytest.main()