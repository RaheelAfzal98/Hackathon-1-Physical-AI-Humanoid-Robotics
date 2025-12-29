import asyncio
import logging
from typing import List, Dict, Any
from app.services.url_crawler import get_all_urls
from app.services.text_extractor import extract_text_from_url
from app.services.chunker import chunk_text
from app.services.embedding_service import embed
from app.services.vector_storage import create_collection, save_chunk_to_qdrant
from app.config.settings import settings
from app.utils.id_generator import generate_content_hash


# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def execute_ingestion_pipeline(
    base_url: str = settings.source_base_url,
    collection_name: str = settings.qdrant_collection_name
):
    """
    Execute the complete ingestion pipeline from URL discovery to vector storage
    """
    logger.info(f"Starting ingestion pipeline for: {base_url}")

    # Step 1: Discover all accessible URLs
    urls = get_all_urls(base_url)
    logger.info(f"Discovered {len(urls)} URLs to process")

    # Step 2: Create Qdrant collection if it doesn't exist
    create_collection(collection_name, vector_size=settings.embedding_dimension)
    logger.info(f"Collection '{collection_name}' prepared")

    total_chunks = 0
    processed_urls = 0
    errors = []

    # Step 3: Process each URL
    for i, url in enumerate(urls):
        logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")

        try:
            # Extract clean text from the URL
            text_data = extract_text_from_url(url)

            # Perform content change detection to avoid reprocessing unchanged content
            content_hash = generate_content_hash(text_data['text'])

            # We would check if content has changed by querying Qdrant for the URL
            # For now, we'll implement a basic check based on the extracted content hash
            # In a full implementation, you would query Qdrant for existing content with this URL
            # and compare the stored hash with the current content hash

            # Chunk the text content
            chunks = chunk_text(
                text_content=text_data['text'],
                chunk_size=settings.chunk_size,
                chunk_overlap=settings.chunk_overlap
            )

            # Generate embeddings for each chunk
            text_list = [chunk['text'] for chunk in chunks]
            if text_list:  # Only generate embeddings if there's text to process
                embeddings = embed(text_list, model=settings.embedding_model)

                # Store each chunk with its embedding in Qdrant
                for j, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                    chunk_data = {
                        **chunk,
                        'source_url': url,
                        'title': text_data.get('title', ''),
                        'hash': text_data.get('hash', ''),
                        'created_at': text_data.get('created_at', ''),
                        'updated_at': text_data.get('updated_at', '')
                    }

                    chunk_id = save_chunk_to_qdrant(
                        chunk_data=chunk_data,
                        collection_name=collection_name,
                        vector=embedding
                    )

                    total_chunks += 1
                    logger.debug(f"  Saved chunk {j+1}/{len(chunks)} with ID: {chunk_id[:8]}...")

            processed_urls += 1
            # Progress update every 10 URLs
            if (i + 1) % 10 == 0:
                logger.info(f"Progress: Processed {i+1}/{len(urls)} URLs")

        except Exception as e:
            error_msg = f"Error processing URL {url}: {str(e)}"
            logger.error(error_msg)
            errors.append(error_msg)
            continue

    stats = {
        'urls_processed': processed_urls,
        'total_urls_found': len(urls),
        'chunks_created': total_chunks,
        'collection_name': collection_name,
        'errors': errors
    }

    logger.info(f"\nPipeline completed!")
    logger.info(f"Processed {processed_urls}/{len(urls)} URLs")
    logger.info(f"Created {total_chunks} content chunks")
    logger.info(f"All vectors stored in collection: {collection_name}")

    if errors:
        logger.warning(f"Encountered {len(errors)} errors during processing")

    return stats


if __name__ == "__main__":
    # Run the ingestion pipeline
    stats = execute_ingestion_pipeline()
    logger.info("Pipeline Statistics:")
    logger.info(stats)