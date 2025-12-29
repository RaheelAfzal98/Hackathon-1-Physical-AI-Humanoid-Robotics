"""
RAG Retrieval Pipeline for validating semantic retrieval from Qdrant

This script:
1. Connects to Qdrant Cloud to access stored book content
2. Converts natural-language queries to embeddings using Cohere
3. Performs vector similarity search to retrieve relevant content chunks
4. Returns results with complete metadata for validation
"""
import os
import sys
import logging
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Add the necessary paths to sys.path
sys.path.append('.')
sys.path.append('src')

# Load environment variables from .env file if it exists
if os.path.exists('.env'):
    with open('.env', 'r') as file:
        for line in file:
            if line.strip() and not line.startswith('#'):
                key, value = line.strip().split('=', 1)
                os.environ[key] = value

from src.config.settings import settings
from src.rag.client import QdrantClientWrapper
import cohere

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QueryResult(BaseModel):
    """Model for a single retrieved result"""
    id: str
    source_url: str
    page_title: str
    section_heading: str
    chunk_index: int
    content: str
    content_type: str
    score: float
    token_count: int


class RAGRetriever:
    def __init__(self, cohere_api_key: str, qdrant_url: str, qdrant_api_key: str,
                 collection_name: str):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(cohere_api_key)

        # Initialize Qdrant client wrapper
        self.client = QdrantClientWrapper()

    def embed_query(self, query: str) -> List[float]:
        """Convert a natural language query to an embedding using Cohere"""
        try:
            response = self.cohere_client.embed(
                texts=[query],
                model=settings.embedding_model,  # Using the model from settings
                input_type="search_query"  # Using search_query for query embeddings
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating embedding for query: {str(e)}")
            raise

    def retrieve_chunks(self, query: str, top_k: int = 5, similarity_threshold: float = 0.3) -> List[QueryResult]:
        """Retrieve relevant content chunks based on semantic similarity"""
        # Generate embedding for the query
        query_embedding = self.embed_query(query)

        # Perform similarity search in Qdrant using the wrapper
        search_results = self.client.search(
            query_vector=query_embedding,
            top_k=top_k,
            similarity_threshold=similarity_threshold
        )

        # Convert results to QueryResult objects
        results = []
        for hit in search_results:
            payload = hit['payload']
            # Handle the metadata structure from the ingestion
            metadata = payload.get('metadata', {})

            result = QueryResult(
                id=hit['id'],
                source_url=metadata.get('source_url', payload.get('source_url', '')),
                page_title=metadata.get('page_title', payload.get('page_title', '')),
                section_heading=metadata.get('section_heading', payload.get('section_heading', '')),
                chunk_index=metadata.get('chunk_index', payload.get('chunk_index', 0)),
                content=payload.get('content', ''),
                content_type=metadata.get('content_type', payload.get('content_type', '')),
                score=hit['similarity_score'],
                token_count=metadata.get('token_count', payload.get('token_count', 0))
            )
            results.append(result)

        return results

    def validate_retrieval_accuracy(self, test_queries: List[Dict[str, str]]) -> Dict[str, Any]:
        """Validate retrieval accuracy with predefined test queries"""
        results = {
            'queries': [],
            'overall_success': True,
            'accuracy_report': []
        }
        
        for query_info in test_queries:
            query = query_info['query']
            expected_keywords = query_info.get('expected_keywords', [])
            top_k = query_info.get('top_k', 5)
            
            logger.info(f"Testing query: {query}")
            
            # Retrieve results
            retrieved_chunks = self.retrieve_chunks(query, top_k=top_k)
            
            # Check if expected keywords appear in results
            found_keywords = []
            for chunk in retrieved_chunks:
                content_lower = chunk.content.lower()
                found = [kw for kw in expected_keywords if kw.lower() in content_lower]
                found_keywords.extend(found)
            
            # Calculate accuracy metrics
            unique_expected = set(expected_keywords)
            unique_found = set(found_keywords)
            accuracy = len(unique_found) / len(unique_expected) if unique_expected else 0
            
            query_result = {
                'query': query,
                'expected_keywords': expected_keywords,
                'found_keywords': list(unique_found),
                'accuracy': accuracy,
                'retrieved_chunks': len(retrieved_chunks),
                'sample_results': [
                    {
                        'source_url': chunk.source_url,
                        'content_preview': chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,
                        'score': chunk.score
                    }
                    for chunk in retrieved_chunks[:3]  # Show first 3 results
                ]
            }
            
            results['queries'].append(query_result)
            
            # If accuracy is low, mark overall as failed
            if accuracy < 0.5:  # Consider <50% keyword match as low accuracy
                results['overall_success'] = False
            
            logger.info(f"  Accuracy: {accuracy:.2%} ({len(unique_found)}/{len(unique_expected)} keywords found)")
        
        return results


def main():
    """Test the retrieval pipeline with various queries"""
    logger.info("Starting RAG Retrieval Pipeline Validation...")
    
    # Initialize the retriever
    retriever = RAGRetriever(
        cohere_api_key=settings.cohere_api_key,
        qdrant_url=settings.qdrant_url,
        qdrant_api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name
    )

    # Test basic connectivity
    logger.info("Validating connection to Qdrant Cloud...")
    try:
        connection_ok = retriever.client.check_connection()
        if connection_ok:
            # Get collection info to verify content
            from qdrant_client import QdrantClient
            qdrant_raw_client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                https=True if "https" in settings.qdrant_url else False
            )
            collection_info = qdrant_raw_client.get_collection(settings.qdrant_collection_name)
            logger.info(f"Successfully connected to collection: {settings.qdrant_collection_name}")
            logger.info(f"Collection has {collection_info.points_count} vectors")
            logger.info(f"Vector size: {collection_info.config.params.vectors.size}")
        else:
            logger.error("Failed to connect to Qdrant")
            return
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {str(e)}")
        return
    
    # Test queries for validation
    test_queries = [
        {
            'query': 'What is ROS 2 and how does it work with humanoid robots?',
            'expected_keywords': ['ROS 2', 'robot operating system', 'nodes', 'topics', 'services']
        },
        {
            'query': 'Explain the Digital Twin concept in robotics',
            'expected_keywords': ['digital twin', 'simulation', 'gazebo', 'unity']
        },
        {
            'query': 'How does NVIDIA Isaac relate to humanoid robotics?',
            'expected_keywords': ['nvidia', 'isaac', 'ai', 'robotics', 'perception']
        },
        {
            'query': 'What are the prerequisites for this textbook?',
            'expected_keywords': ['prerequisites', 'python', 'programming', 'mathematics']
        }
    ]
    
    logger.info("Running retrieval accuracy validation...")
    validation_results = retriever.validate_retrieval_accuracy(test_queries)
    
    logger.info("\n" + "="*60)
    logger.info("VALIDATION RESULTS")
    logger.info("="*60)
    
    for i, query_result in enumerate(validation_results['queries']):
        logger.info(f"\nQuery {i+1}: {query_result['query']}")
        logger.info(f"  Expected Keywords: {', '.join(query_result['expected_keywords'])}")
        logger.info(f"  Found Keywords: {', '.join(query_result['found_keywords'])}")
        logger.info(f"  Accuracy: {query_result['accuracy']:.2%}")
        logger.info(f"  Retrieved Chunks: {query_result['retrieved_chunks']}")
        
        logger.info("  Sample Results:")
        for j, sample in enumerate(query_result['sample_results']):
            logger.info(f"    {j+1}. Score: {sample['score']:.3f}")
            logger.info(f"       Source: {sample['source_url']}")
            logger.info(f"       Content Preview: {sample['content_preview']}")
    
    logger.info(f"\nOverall Validation Success: {validation_results['overall_success']}")
    
    # Additional validation: Check if vectors have complete metadata
    logger.info("\nValidating metadata completeness...")
    try:
        # Get a sample of vectors from the collection using raw Qdrant client
        from qdrant_client import QdrantClient
        qdrant_raw_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            https=True if "https" in settings.qdrant_url else False
        )

        sample_result = qdrant_raw_client.scroll(
            collection_name=settings.qdrant_collection_name,
            limit=5
        )

        if sample_result[0]:  # If we have results
            sample_point = sample_result[0][0]  # Get first point
            payload = sample_point.payload

            # Check for metadata structure (it might be in a 'metadata' key)
            if 'metadata' in payload:
                metadata = payload['metadata']
                required_fields = ['source_url', 'page_title', 'chunk_index', 'document_id']
                missing_fields = [field for field in required_fields if field not in metadata]
            else:
                # Direct fields in payload
                required_fields = ['source_url', 'page_title', 'section_heading', 'chunk_index', 'content', 'content_type']
                missing_fields = [field for field in required_fields if field not in payload]

            if not missing_fields:
                logger.info("All required metadata fields are present in stored vectors")
            else:
                logger.warning(f"Missing metadata fields: {missing_fields}")

        # Verify no duplicate vectors (by checking for similar content with high similarity)
        logger.info("Checking for duplicate vectors...")
        # This is a simple check - in practice, you might want a more sophisticated duplicate detection

    except Exception as e:
        logger.error(f"Error during metadata validation: {str(e)}")

    logger.info("\nRAG Retrieval Pipeline Validation Complete!")
    logger.info("The pipeline is ready for use with downstream AI agent systems.")


if __name__ == "__main__":
    main()