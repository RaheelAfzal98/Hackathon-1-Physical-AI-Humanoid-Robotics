import pytest
from unittest.mock import patch, Mock
from fastapi.testclient import TestClient
from src.main import app
from src.rag.client import QdrantClientWrapper
from src.rag.models import QueryRequest, RetrievalResponse, PipelineValidationResponse


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
        mock_instance.collection_name = "book_embeddings"
        yield mock_instance


def test_retrieve_endpoint_contract(client, mock_qdrant_client):
    """Test that the retrieve endpoint conforms to the OpenAPI contract"""
    # Arrange
    request_payload = {
        "text": "test query",
        "top_k": 5,
        "similarity_threshold": 0.7
    }
    
    # Mock search results that conform to the expected contract
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
    with patch('src.rag.retrieval.process_query_embedding', return_value=[0.1, 0.2, 0.3, 0.4, 0.5]):
        # Act
        response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert - Check the response conforms to the contract
    assert response.status_code == 200
    response_data = response.json()
    
    # Validate response structure according to the RetrievalResponse model contract
    assert "query_text" in response_data
    assert "results" in response_data
    assert "total_chunks_processed" in response_data
    assert "search_time_ms" in response_data
    assert "retrieval_parameters" in response_data
    
    # Validate types according to contract
    assert isinstance(response_data["query_text"], str)
    assert isinstance(response_data["results"], list)
    assert isinstance(response_data["total_chunks_processed"], int)
    assert isinstance(response_data["search_time_ms"], (int, float))
    assert isinstance(response_data["retrieval_parameters"], dict)
    
    # Validate nested structure
    if response_data["results"]:
        result = response_data["results"][0]
        assert "chunk" in result
        assert "similarity_score" in result
        assert "rank" in result
        
        chunk = result["chunk"]
        assert "id" in chunk
        assert "content" in chunk
        assert "metadata" in chunk
        
        metadata = chunk["metadata"]
        assert "source_url" in metadata
        assert "page_title" in metadata
        assert "document_id" in metadata


def test_retrieve_endpoint_validation_error_contract(client):
    """Test that the retrieve endpoint returns proper validation errors according to contract"""
    # Arrange - Invalid request that should trigger validation error
    request_payload = {
        "text": "",  # Invalid: empty text
        "top_k": 5,
        "similarity_threshold": 0.7
    }
    
    # Act
    response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert - Should return 422 for validation error
    assert response.status_code == 422
    
    # Check that the error response structure conforms to contract
    response_data = response.json()
    assert "detail" in response_data  # FastAPI standard validation error format


def test_retrieve_endpoint_required_fields_contract(client, mock_qdrant_client):
    """Test that the retrieve endpoint properly validates required fields according to contract"""
    # Arrange - Request without required 'text' field
    request_payload = {
        "top_k": 5,
        "similarity_threshold": 0.7
    }
    
    # Act
    response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert - Should return 422 for validation error
    assert response.status_code == 422


def test_validate_pipeline_endpoint_contract(client, mock_qdrant_client):
    """Test that the validate-pipeline endpoint conforms to the OpenAPI contract"""
    # Act
    response = client.get("/rag/validate-pipeline")
    
    # Assert - Check the response conforms to the contract
    assert response.status_code == 200
    response_data = response.json()
    
    # Validate response structure according to the PipelineValidationResponse model
    assert "status" in response_data
    assert "message" in response_data
    assert "results" in response_data
    
    # Validate types according to contract
    assert isinstance(response_data["status"], str)
    assert isinstance(response_data["message"], str)
    assert isinstance(response_data["results"], list)
    
    # Validate status field values
    assert response_data["status"] in ["success", "failure"]


def test_retrieve_endpoint_optional_fields_contract(client, mock_qdrant_client):
    """Test that optional fields in the retrieve endpoint work according to contract"""
    # Arrange - Request with only required field
    request_payload = {
        "text": "minimal query"
    }
    
    # Mock the embedding generation
    with patch('src.rag.retrieval.process_query_embedding', return_value=[0.1, 0.2, 0.3, 0.4, 0.5]):
        # Act
        response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert - Should work with just the required text field
    assert response.status_code == 200
    
    response_data = response.json()
    # Should have defaults for optional fields
    assert response_data["retrieval_parameters"]["top_k"] == 5
    assert response_data["retrieval_parameters"]["similarity_threshold"] == 0.5


def test_retrieve_endpoint_parameter_bounds_contract(client, mock_qdrant_client):
    """Test that parameter bounds are validated according to contract"""
    # Arrange - Request with out of bounds parameters
    request_payload = {
        "text": "test query",
        "top_k": 30,  # Out of bounds: max 20
        "similarity_threshold": 1.5  # Out of bounds: max 1.0
    }
    
    # Act
    response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert - Should return 422 for validation error
    assert response.status_code == 422


def test_api_response_schema_compliance(client, mock_qdrant_client):
    """Test comprehensive response schema compliance with OpenAPI contract"""
    # Arrange
    request_payload = {
        "text": "compliance test query",
        "top_k": 2,
        "similarity_threshold": 0.6,
        "filters": {"category": "test"}
    }
    
    # Mock search results
    mock_search_results = [
        {
            "id": "chunk1",
            "payload": {
                "content": "First relevant content",
                "metadata": {
                    "source_url": "/test/doc1.md",
                    "page_title": "Document 1",
                    "document_id": "doc1",
                    "section": "Introduction",
                    "chunk_index": 0
                }
            },
            "similarity_score": 0.88
        },
        {
            "id": "chunk2",
            "payload": {
                "content": "Second relevant content", 
                "metadata": {
                    "source_url": "/test/doc1.md",
                    "page_title": "Document 1", 
                    "document_id": "doc1",
                    "section": "Methods",
                    "chunk_index": 1
                }
            },
            "similarity_score": 0.75
        }
    ]
    
    mock_qdrant_client.search.return_value = mock_search_results
    
    # Mock the embedding generation
    with patch('src.rag.retrieval.process_query_embedding', return_value=[0.1, 0.2, 0.3, 0.4, 0.5]):
        # Act
        response = client.post("/rag/retrieve", json=request_payload)
    
    # Assert
    assert response.status_code == 200
    response_data = response.json()
    
    # Check full schema compliance
    expected_top_level_fields = {"query_text", "results", "total_chunks_processed", "search_time_ms", "retrieval_parameters"}
    assert set(response_data.keys()) >= expected_top_level_fields
    
    # Check results schema
    assert len(response_data["results"]) == 2
    for result in response_data["results"]:
        assert set(result.keys()) >= {"chunk", "similarity_score", "rank"}
        assert isinstance(result["similarity_score"], (int, float))
        assert isinstance(result["rank"], int)
        
        # Check chunk schema
        chunk = result["chunk"]
        assert set(chunk.keys()) >= {"id", "content", "metadata"}
        assert isinstance(chunk["id"], str)
        assert isinstance(chunk["content"], str)
        assert isinstance(chunk["metadata"], dict)
        
        # Check metadata schema
        metadata = chunk["metadata"]
        assert set(metadata.keys()) >= {"source_url", "page_title", "document_id"}
        assert isinstance(metadata["source_url"], str)
        assert isinstance(metadata["page_title"], str)
        assert isinstance(metadata["document_id"], str)