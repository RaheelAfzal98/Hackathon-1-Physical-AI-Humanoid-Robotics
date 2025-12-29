#!/usr/bin/env python3
"""
Script to ingest your AI book content into Qdrant database
"""

import os
import sys
import requests
from urllib.parse import urljoin, urlparse

# Add src to path to import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Load environment variables from .env file if it exists
if os.path.exists('.env'):
    with open('.env', 'r') as file:
        for line in file:
            if line.strip() and not line.startswith('#'):
                key, value = line.strip().split('=', 1)
                os.environ[key] = value

from src.config.settings import settings
from src.rag.ingestion import IngestionPipeline


def main():
    print("Starting ingestion of AI book content into Qdrant...")
    
    # Verify we can access the settings
    print(f"Using Qdrant collection: {settings.qdrant_collection_name}")
    print(f"Target URLs will be crawled from your source base URL")
    
    # Define the URLs to crawl - based on your base URL
    # For a Docusaurus site, common paths for documentation are typically /docs, /guides, etc.
    base_url = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app"
    
    # Common paths for a Docusaurus-based book/ documentation site
    urls_to_crawl = [
        f"{base_url}",  # Homepage
        f"{base_url}/docs",  # Documentation section
        # Add more specific paths if needed
    ]
    
    print(f"Crawling the following URLs: {urls_to_crawl}")
    
    try:
        # Create ingestion pipeline with your settings
        ingestion_pipeline = IngestionPipeline(
            cohere_api_key=settings.cohere_api_key,
            qdrant_url=settings.qdrant_url,
            qdrant_api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection_name
        )
        
        print("Starting ingestion process...")
        
        # Run the ingestion
        ingestion_pipeline.run_ingestion(
            urls=urls_to_crawl,
            chunk_size=512,  # Appropriate for technical content
            overlap=50
        )
        
        print("\n✅ Ingestion completed successfully!")
        print(f"Your book content has been embedded and stored in Qdrant collection: {settings.qdrant_collection_name}")
        print("You can now use the retrieval API to search through your book content.")
        
    except Exception as e:
        print(f"❌ Error during ingestion: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()