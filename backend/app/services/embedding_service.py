import asyncio
import time
import random
import cohere
from typing import List
from ..config.settings import settings

# Initialize Cohere client
co = cohere.Client(settings.cohere_api_key)

def embed(text_chunks: List[str], model: str = 'embed-multilingual-v3.0') -> List[List[float]]:
    """
    Generate embeddings for text chunks using Cohere API
    Implements exponential backoff and retry logic
    """
    last_exception = None

    for attempt in range(settings.MAX_RETRIES + 1):
        try:
            # Use Cohere's embed function to generate embeddings
            response = co.embed(
                texts=text_chunks,
                model=model,
                input_type="search_document"  # Using search_document for RAG use case
            )

            # Extract embeddings from the response
            embeddings = [embedding for embedding in response.embeddings]

            return embeddings
        except Exception as e:
            last_exception = e
            if attempt < settings.max_retries:
                # Exponential backoff with jitter
                delay = settings.delay_base * (2 ** attempt) + random.uniform(0, 1)
                time.sleep(delay)
            else:
                # All retry attempts failed
                raise Exception(f"Cohere API error after {settings.max_retries} retries: {str(e)}")

    # This should not be reached, but included for completeness
    raise last_exception