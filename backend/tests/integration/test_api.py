import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, Mock
from src.main import app
from src.rag.client import QdrantClientWrapper
from src.rag.embedding import CohereEmbeddingService
from src.rag.models import QueryRequest


@pytest.fixture
def client():
    """Create a test client for the API"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client for testing"""
    with patch('src.rag.client.QdrantClientWrapper') as mock:
        mock_instance = Mock(spec=QdrantClientWrapper)
        mock.return_value = mock_instance
        mock_instance.search.return_value = []
        mock_instance.check_connection.return_value = True
        yield mock_instance


@pytest.fixture
def mock_cohere_service():
    """Mock Cohere service for testing"""
    with patch('src.rag.embedding.CohereEmbeddingService') as mock:
        mock_instance = Mock(spec=CohereEmbeddingService)
        mock.return_value = mock_instance
        mock_instance.get_embedding.return_value = [0.1, 0.2, 0.3, 0.4]
        yield mock_instance


def test_retrieve_endpoint_basic_query(client, mock_qdrant_client, mock_cohere_service):
    """Test the retrieve endpoint with a basic query"""
    # Arrange
    request_payload = {
        "text": "test query"
    }
    
    # Mock the embedding generation
    with patch('src.rag.retrieval.process_query_embedding', return_value=[0.1, 0.2, 0.3, 0.4]):
        # Act
        response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert
    assert response.status_code == 200
    response_data = response.json()
    
    assert response_data["query_text"] == "test query"
    assert response_data["results"] == []
    assert response_data["total_chunks_processed"] == 0
    assert "search_time_ms" in response_data
    assert response_data["retrieval_parameters"]["top_k"] == 5
    assert response_data["retrieval_parameters"]["similarity_threshold"] == 0.5


def test_retrieve_endpoint_with_parameters(client, mock_qdrant_client, mock_cohere_service):
    """Test the retrieve endpoint with custom parameters"""
    # Arrange
    request_payload = {
        "text": "test query",
        "top_k": 3,
        "similarity_threshold": 0.8
    }
    
    # Mock the embedding generation
    with patch('src.rag.retrieval.process_query_embedding', return_value=[0.1, 0.2, 0.3, 0.4]):
        # Act
        response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert
    assert response.status_code == 200
    response_data = response.json()
    
    assert response_data["retrieval_parameters"]["top_k"] == 3
    assert response_data["retrieval_parameters"]["similarity_threshold"] == 0.8


def test_retrieve_endpoint_with_search_results(client, mock_qdrant_client, mock_cohere_service):
    """Test the retrieve endpoint with actual search results"""
    # Arrange
    request_payload = {
        "text": "test query"
    }
    
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
    with patch('src.rag.retrieval.process_query_embedding', return_value=[0.1, 0.2, 0.3, 0.4]):
        # Act
        response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert
    assert response.status_code == 200
    response_data = response.json()
    
    assert response_data["total_chunks_processed"] == 1
    assert len(response_data["results"]) == 1
    assert response_data["results"][0]["chunk"]["id"] == "chunk1"
    assert response_data["results"][0]["chunk"]["content"] == "This is relevant content"
    assert response_data["results"][0]["similarity_score"] == 0.85
    assert response_data["results"][0]["rank"] == 0
    assert response_data["results"][0]["chunk"]["metadata"]["source_url"] == "/test/source.md"


def test_retrieve_endpoint_validation_error(client):
    """Test the retrieve endpoint with invalid input"""
    # Arrange
    request_payload = {
        "text": "",  # Invalid: empty text
        "top_k": 5,
        "similarity_threshold": 0.5
    }
    
    # Act
    response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert
    assert response.status_code == 422  # Validation error


def test_retrieve_endpoint_internal_error(client, mock_qdrant_client):
    """Test the retrieve endpoint with internal error"""
    # Arrange
    request_payload = {
        "text": "test query"
    }
    
    # Mock an error in the Qdrant client
    mock_qdrant_client.search.side_effect = Exception("Connection failed")
    
    # Mock the embedding generation
    with patch('src.rag.retrieval.process_query_embedding', return_value=[0.1, 0.2, 0.3, 0.4]):
        # Act
        response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert
    assert response.status_code == 500  # Internal server error


def test_validate_pipeline_endpoint(client, mock_qdrant_client):
    """Test the validate pipeline endpoint"""
    # Arrange
    mock_qdrant_client.check_connection.return_value = True
    
    # Act
    response = client.get("/rag/validate-pipeline")
    
    # Assert
    assert response.status_code == 200
    response_data = response.json()
    
    assert response_data["status"] in ["success", "failure"]  # Depends on other checks
    assert "message" in response_data
    assert "results" in response_data
    
    # Check that qdrant connectivity test is included
    qdrant_test = next((test for test in response_data["results"] if test["test_name"] == "qdrant_connectivity"), None)
    assert qdrant_test is not None
    assert qdrant_test["status"] == "pass"


def test_root_endpoint(client):
    """Test the root endpoint"""
    # Act
    response = client.get("/")
    
    # Assert
    assert response.status_code == 200
    response_data = response.json()
    
    assert "message" in response_data
    assert "version" in response_data