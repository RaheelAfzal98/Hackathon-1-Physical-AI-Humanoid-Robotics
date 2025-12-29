import cohere
from typing import List
from src.config.settings import settings
import logging


class CohereEmbeddingService:
    def __init__(self):
        self.client = cohere.Client(settings.cohere_api_key)
        self.model = settings.embedding_model
        self.logger = logging.getLogger(__name__)

    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.
        
        Args:
            texts: List of texts to generate embeddings for
            
        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_query"  # Using search_query for retrieval queries
            )
            
            embeddings = [embedding for embedding in response.embeddings]
            return embeddings
        except Exception as e:
            self.logger.error(f"Error generating embeddings with Cohere: {str(e)}")
            raise e

    def get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using Cohere.
        
        Args:
            text: Text to generate embedding for
            
        Returns:
            Embedding as a list of floats
        """
        embeddings = self.get_embeddings([text])
        return embeddings[0] if embeddings else []


def get_embeddings_for_texts(texts: List[str]) -> List[List[float]]:
    """
    Utility function to get embeddings for a list of texts.
    Uses the Cohere service to convert natural language texts into vector embeddings.
    """
    cohere_service = CohereEmbeddingService()
    return cohere_service.get_embeddings(texts)


def get_embedding_for_text(text: str) -> List[float]:
    """
    Utility function to get embedding for a single text.
    Uses the Cohere service to convert a natural language query into a vector embedding.
    """
    cohere_service = CohereEmbeddingService()
    return cohere_service.get_embedding(text)


def process_query_embedding(query_text: str) -> List[float]:
    """
    Process a natural language query and convert it to an embedding vector.

    Args:
        query_text: The natural language query text

    Returns:
        A list of floats representing the embedding vector
    """
    try:
        # Validate query text
        if not query_text or len(query_text.strip()) == 0:
            raise ValueError("Query text cannot be empty")

        # Generate embedding using Cohere
        embedding = get_embedding_for_text(query_text)

        # Validate that we got a proper embedding
        if not embedding or len(embedding) == 0:
            raise ValueError("Failed to generate embedding for query")

        return embedding
    except Exception as e:
        logging.error(f"Error processing query embedding: {str(e)}")
        raise e