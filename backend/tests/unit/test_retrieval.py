import pytest
from unittest.mock import Mock, patch
from src.rag.models import QueryRequest
from src.rag.retrieval import RetrievalService
from src.rag.client import QdrantClientWrapper
from src.rag.embedding import CohereEmbeddingService


class TestRetrievalService:
    @pytest.fixture
    def mock_qdrant_client(self):
        """Mock Qdrant client for testing"""
        with patch('src.rag.client.QdrantClientWrapper') as mock:
            mock_instance = Mock(spec=QdrantClientWrapper)
            mock.return_value = mock_instance
            mock_instance.search.return_value = []
            mock_instance.check_connection.return_value = True
            mock_instance.collection_name = "book_embeddings"  # Add the missing attribute
            yield mock_instance

    @pytest.fixture
    def mock_cohere_service(self):
        """Mock Cohere service for testing"""
        with patch('src.rag.embedding.CohereEmbeddingService') as mock:
            mock_instance = Mock(spec=CohereEmbeddingService)
            mock.return_value = mock_instance
            mock_instance.get_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
            yield mock_instance

    @pytest.fixture
    def retrieval_service(self, mock_qdrant_client, mock_cohere_service):
        """Create retrieval service with mocked dependencies"""
        service = RetrievalService()
        service.qdrant_client = mock_qdrant_client
        return service

    def test_retrieve_basic_query(self, retrieval_service, mock_qdrant_client):
        """Test basic retrieval with a simple query"""
        # Arrange
        query_request = QueryRequest(text="test query")
        expected_embedding = [0.1, 0.2, 0.3, 0.4]
        
        # Mock the embedding generation
        with patch('src.rag.retrieval.process_query_embedding', return_value=expected_embedding):
            # Act
            result = retrieval_service.retrieve(query_request)
        
        # Assert
        assert result.query_text == "test query"
        assert result.results == []
        assert result.total_chunks_processed == 0
        assert result.search_time_ms >= 0
        assert result.retrieval_parameters.top_k == 5
        assert result.retrieval_parameters.similarity_threshold == 0.5

    def test_retrieve_with_custom_parameters(self, retrieval_service, mock_qdrant_client):
        """Test retrieval with custom top_k and similarity_threshold"""
        # Arrange
        query_request = QueryRequest(
            text="test query", 
            top_k=3, 
            similarity_threshold=0.8
        )
        expected_embedding = [0.1, 0.2, 0.3, 0.4]
        
        # Mock the embedding generation
        with patch('src.rag.retrieval.process_query_embedding', return_value=expected_embedding):
            # Act
            result = retrieval_service.retrieve(query_request)
        
        # Assert
        assert result.retrieval_parameters.top_k == 3
        assert result.retrieval_parameters.similarity_threshold == 0.8

    def test_retrieve_with_search_results(self, retrieval_service, mock_qdrant_client):
        """Test retrieval with actual search results"""
        # Arrange
        query_request = QueryRequest(text="test query")
        expected_embedding = [0.1, 0.2, 0.3, 0.4]
        
        # Mock search results
        mock_search_results = [
            {
                "id": "chunk1",
                "payload": {
                    "content": "This is relevant content",
                    "metadata": {
                        "source_url": "/test/source.md",
                        "page_title": "Test Page",
                        "document_id": "doc1"
                    }
                },
                "similarity_score": 0.85
            }
        ]
        
        mock_qdrant_client.search.return_value = mock_search_results
        
        # Mock the embedding generation
        with patch('src.rag.retrieval.process_query_embedding', return_value=expected_embedding):
            # Act
            result = retrieval_service.retrieve(query_request)
        
        # Assert
        assert result.total_chunks_processed == 1
        assert len(result.results) == 1
        assert result.results[0].chunk.id == "chunk1"
        assert result.results[0].chunk.content == "This is relevant content"
        assert result.results[0].similarity_score == 0.85
        assert result.results[0].rank == 0

    def test_validate_connection_success(self, retrieval_service, mock_qdrant_client):
        """Test successful connection validation"""
        # Arrange
        mock_qdrant_client.check_connection.return_value = True
        
        # Act
        result = retrieval_service.validate_connection()
        
        # Assert
        assert result is True

    def test_validate_connection_failure(self, retrieval_service, mock_qdrant_client):
        """Test failed connection validation"""
        # Arrange
        mock_qdrant_client.check_connection.return_value = False
        
        # Act
        result = retrieval_service.validate_connection()
        
        # Assert
        assert result is False


class TestQueryValidation:
    def test_query_request_valid(self):
        """Test that valid query requests pass validation"""
        query = QueryRequest(text="valid query", top_k=5, similarity_threshold=0.7)
        
        assert query.text == "valid query"
        assert query.top_k == 5
        assert query.similarity_threshold == 0.7

    def test_query_request_text_validation(self):
        """Test text validation fails for empty text"""
        with pytest.raises(ValueError):
            QueryRequest(text="", top_k=5, similarity_threshold=0.7)

    def test_query_request_top_k_validation(self):
        """Test top_k validation fails for invalid values"""
        with pytest.raises(ValueError):
            QueryRequest(text="test", top_k=0, similarity_threshold=0.7)
        
        with pytest.raises(ValueError):
            QueryRequest(text="test", top_k=25, similarity_threshold=0.7)

    def test_query_request_similarity_threshold_validation(self):
        """Test similarity_threshold validation fails for invalid values"""
        with pytest.raises(ValueError):
            QueryRequest(text="test", top_k=5, similarity_threshold=-0.1)
        
        with pytest.raises(ValueError):
            QueryRequest(text="test", top_k=5, similarity_threshold=1.5)