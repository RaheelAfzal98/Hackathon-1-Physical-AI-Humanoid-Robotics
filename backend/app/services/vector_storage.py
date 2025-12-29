import time
import random
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import Dict, List, Any
from ..config.settings import settings

# Initialize Qdrant client
client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
)

def create_collection(collection_name: str, vector_size: int, distance: str = 'Cosine') -> bool:
    """
    Create a Qdrant collection to store embeddings
    Idempotent operation that doesn't fail if collection already exists
    Implements retry logic with exponential backoff
    """
    last_exception = None

    for attempt in range(settings.MAX_RETRIES + 1):
        try:
            # Check if collection already exists
            collections = client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if collection_name in collection_names:
                return True  # Collection already exists

            # Create new collection
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance[distance.upper()]  # Cosine, Euclid, Dot
                )
            )

            return True
        except Exception as e:
            last_exception = e
            if attempt < settings.max_retries:
                # Exponential backoff with jitter
                delay = settings.delay_base * (2 ** attempt) + random.uniform(0, 1)
                time.sleep(delay)
            else:
                # All retry attempts failed
                raise Exception(f"Qdrant API error after {settings.max_retries} retries: {str(e)}")

    # This should not be reached, but included for completeness
    raise last_exception

def save_chunk_to_qdrant(chunk_data: Dict[str, Any], collection_name: str, vector: List[float]) -> str:
    """
    Save a content chunk with its embedding to Qdrant
    Uses SHA256 hash of URL+content+index as the point ID for idempotency
    Implements retry logic with exponential backoff
    """
    last_exception = None

    for attempt in range(settings.MAX_RETRIES + 1):
        try:
            point_id = chunk_data.get('id')
            if not point_id:
                # Generate ID if not provided
                from ..utils.id_generator import generate_chunk_id
                point_id = generate_chunk_id(
                    chunk_data['source_url'],
                    chunk_data['text_content'][:100],  # First 100 chars of content
                    chunk_data['chunk_index']
                )

            # Prepare the payload with chunk data
            payload = {
                "id": point_id,
                "text_content": chunk_data['text_content'],
                "source_url": chunk_data['source_url'],
                "title": chunk_data.get('title', ''),
                "created_at": chunk_data.get('created_at', ''),
                "updated_at": chunk_data.get('updated_at', ''),
                "chunk_index": chunk_data['chunk_index'],
                "total_chunks": chunk_data['total_chunks'],
                "hash": chunk_data.get('hash', ''),
                "metadata": {
                    "source_domain": chunk_data['source_url'].split('/')[2] if '//' in chunk_data['source_url'] else '',
                    "word_count": len(chunk_data['text_content'].split()),
                    "language": "en"  # Could be detected in the future
                }
            }

            # Upsert the point in Qdrant
            client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,
                        vector=vector,
                        payload=payload
                    )
                ]
            )

            return point_id
        except Exception as e:
            last_exception = e
            if attempt < settings.max_retries:
                # Exponential backoff with jitter
                delay = settings.delay_base * (2 ** attempt) + random.uniform(0, 1)
                time.sleep(delay)
            else:
                # All retry attempts failed
                raise Exception(f"Qdrant API error after {settings.max_retries} retries: {str(e)}")

    # This should not be reached, but included for completeness
    raise last_exception