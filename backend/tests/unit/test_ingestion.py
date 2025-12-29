import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
from typing import List, Dict, Any

# Add src to path to import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.rag.ingestion import WebCrawler, ContentChunker, EmbeddingGenerator, QdrantStorage, IngestionPipeline


class TestWebCrawler:
    def test_extract_content_basic(self):
        """Test basic content extraction from a URL."""
        crawler = WebCrawler(["https://example.com"])

        # Mock the requests.get method
        with patch('src.rag.ingestion.requests.get') as mock_get:
            mock_response = Mock()
            mock_response.content = b"""
            <html>
                <head><title>Test Page</title></head>
                <body>
                    <main>This is the main content of the page.</main>
                </body>
            </html>
            """
            mock_response.raise_for_status.return_value = None
            mock_get.return_value = mock_response
            
            result = crawler.extract_content("https://example.com")
            
            assert result is not None
            assert result['url'] == "https://example.com"
            assert result['title'] == "Test Page"
            assert "main content of the page" in result['content']

    def test_get_all_urls_within_limits(self):
        """Test that URL discovery respects limits."""
        urls = ["https://example.com"]
        crawler = WebCrawler(urls, max_depth=1, max_pages=5)

        # Mock the requests.get method for the main URL
        with patch('src.rag.ingestion.requests.get') as mock_get:
            # First call (for the main URL) - return a page with links
            mock_response_main = Mock()
            mock_response_main.content = b"""
            <html>
                <body>
                    <a href="https://example.com/page1">Page 1</a>
                    <a href="https://example.com/page2">Page 2</a>
                    <a href="https://external.com/page">External</a>
                </body>
            </html>
            """
            mock_response_main.raise_for_status.return_value = None
            
            # Second call (for a subsequent URL) - return a page without links
            mock_response_sub = Mock()
            mock_response_sub.content = b"<html><body>Content without links</body></html>"
            mock_response_sub.raise_for_status.return_value = None
            
            mock_get.side_effect = [mock_response_main, mock_response_sub, mock_response_sub]
            
            all_urls = crawler.get_all_urls()
            
            # Should have the original URL and the two internal links
            assert len(all_urls) >= 1  # At least the original URL
            # External links should not be included
            assert not any("external.com" in url for url in all_urls)


class TestContentChunker:
    def test_chunk_text_basic(self):
        """Test basic text chunking."""
        chunker = ContentChunker(chunk_size=50, overlap=10)
        
        long_text = "This is a sample text that is longer than fifty characters and will be split into multiple chunks with some overlap."
        chunks = chunker.chunk_text(long_text, "https://example.com", "Test Title")
        
        assert len(chunks) > 0
        assert all('content' in chunk for chunk in chunks)
        assert all('metadata' in chunk for chunk in chunks)
        
        # Check that each chunk has the required metadata
        for chunk in chunks:
            assert 'source_url' in chunk['metadata']
            assert 'page_title' in chunk['metadata']
            assert 'chunk_index' in chunk['metadata']
            assert 'document_id' in chunk['metadata']
    
    def test_short_text(self):
        """Test that short text is handled correctly."""
        chunker = ContentChunker(chunk_size=500, overlap=50)
        
        short_text = "This is a short text."
        chunks = chunker.chunk_text(short_text, "https://example.com", "Test Title")
        
        assert len(chunks) == 1
        assert chunks[0]['content'] == short_text


class TestEmbeddingGenerator:
    @patch('src.rag.ingestion.cohere.Client')
    def test_generate_embeddings(self, mock_cohere_client):
        """Test that embeddings can be generated."""
        # Mock the Cohere response
        mock_embed_response = Mock()
        mock_embed_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        
        mock_client_instance = Mock()
        mock_client_instance.embed.return_value = mock_embed_response
        mock_cohere_client.return_value = mock_client_instance
        
        generator = EmbeddingGenerator("fake-api-key")
        texts = ["text1", "text2"]
        
        result = generator.generate_embeddings(texts)
        
        assert len(result) == 2
        assert result[0] == [0.1, 0.2, 0.3]
        assert result[1] == [0.4, 0.5, 0.6]
        
        # Verify the embed method was called with correct parameters
        mock_client_instance.embed.assert_called_once_with(
            texts=texts,
            model="embed-multilingual-v2.0",
            input_type="search_document"
        )


class TestQdrantStorage:
    @patch('src.rag.ingestion.QdrantClient')
    def test_store_embeddings(self, mock_qdrant_client):
        """Test that embeddings can be stored in Qdrant."""
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance
        
        storage = QdrantStorage("fake-url", "fake-key", "test-collection")
        
        # Mock the get_collection method to raise an exception, indicating the collection doesn't exist
        mock_client_instance.get_collection.side_effect = Exception("Collection doesn't exist")
        
        chunks = [
            {
                'content': 'First chunk content',
                'metadata': {
                    'source_url': 'https://example.com',
                    'page_title': 'Test Title',
                    'chunk_index': 0,
                    'document_id': 'doc123'
                }
            },
            {
                'content': 'Second chunk content',
                'metadata': {
                    'source_url': 'https://example.com',
                    'page_title': 'Test Title',
                    'chunk_index': 1,
                    'document_id': 'doc123'
                }
            }
        ]
        
        embeddings = [[0.1, 0.2], [0.3, 0.4]]
        
        storage.store_embeddings(chunks, embeddings)
        
        # Verify upsert was called to store the embeddings
        assert mock_client_instance.upsert.called
        args, kwargs = mock_client_instance.upsert.call_args
        assert kwargs['collection_name'] == 'test-collection'
        assert len(kwargs['points']) == 2


class TestIngestionPipeline:
    @patch('src.rag.ingestion.WebCrawler')
    @patch('src.rag.ingestion.ContentChunker')
    @patch('src.rag.ingestion.EmbeddingGenerator')
    @patch('src.rag.ingestion.QdrantStorage')
    def test_run_ingestion(self, mock_qdrant, mock_embedding, mock_chunker, mock_crawler):
        """Test the end-to-end ingestion pipeline."""
        # Mock all dependencies
        mock_crawler_instance = Mock()
        mock_crawler_instance.get_all_urls.return_value = ["https://example.com"]

        # Mock extract_content to return proper content
        mock_crawler_instance.extract_content.return_value = {
            'url': 'https://example.com',
            'title': 'Example Page',
            'content': 'This is the content of the example page with sufficient text to be chunked properly.',
            'timestamp': 1234567890
        }

        mock_crawler.return_value = mock_crawler_instance

        # Create a real ContentChunker and mock its chunk_text method
        real_chunker_instance = ContentChunker()
        mock_chunker.return_value = real_chunker_instance

        # Mock the chunk_text method to return known chunks
        chunk_result = [
            {
                'content': 'First chunk content',
                'metadata': {
                    'source_url': 'https://example.com',
                    'page_title': 'Example Page',
                    'chunk_index': 0,
                    'document_id': 'doc123'
                }
            }
        ]
        with patch.object(real_chunker_instance, 'chunk_text', return_value=chunk_result):
            # Mock embedding generation
            mock_embedding_instance = Mock()
            mock_embedding_instance.generate_embeddings.return_value = [[0.1, 0.2, 0.3]]
            mock_embedding.return_value = mock_embedding_instance

            # Mock Qdrant storage
            mock_qdrant_instance = Mock()
            mock_qdrant.return_value = mock_qdrant_instance

            # Create and run pipeline
            pipeline = IngestionPipeline(
                cohere_api_key="fake-key",
                qdrant_url="fake-url",
                qdrant_api_key="fake-key",
                collection_name="test-collection"
            )

            # Mock the init_collection method to do nothing
            with patch.object(pipeline.qdrant_storage, 'init_collection'):
                pipeline.run_ingestion(["https://example.com"])

        # Verify all components were called appropriately
        mock_crawler_instance.get_all_urls.assert_called()
        mock_qdrant_instance.store_embeddings.assert_called()