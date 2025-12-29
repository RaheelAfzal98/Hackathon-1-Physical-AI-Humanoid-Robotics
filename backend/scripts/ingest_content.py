#!/usr/bin/env python3
"""
Ingestion Script for RAG System

This script runs the full ingestion pipeline to:
1. Crawl specified website URLs
2. Extract and chunk content
3. Generate embeddings using Cohere
4. Store embeddings in Qdrant vector database
"""

import sys
import os
import argparse
from typing import List

# Add src to path to import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.config.settings import settings
from src.rag.ingestion import IngestionPipeline


def main():
    parser = argparse.ArgumentParser(description='Ingest content into RAG system')
    parser.add_argument('--urls', nargs='+', required=True, help='URLs to crawl and ingest')
    parser.add_argument('--chunk-size', type=int, default=512, help='Size of text chunks (default: 512)')
    parser.add_argument('--overlap', type=int, default=50, help='Overlap between chunks (default: 50)')
    
    args = parser.parse_args()
    
    print(f"Starting ingestion for URLs: {args.urls}")
    print(f"Chunk size: {args.chunk_size}, Overlap: {args.overlap}")
    
    try:
        # Create ingestion pipeline with settings from environment
        ingestion_pipeline = IngestionPipeline(
            cohere_api_key=settings.cohere_api_key,
            qdrant_url=settings.qdrant_url,
            qdrant_api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection_name
        )
        
        # Run the ingestion
        ingestion_pipeline.run_ingestion(
            urls=args.urls,
            chunk_size=args.chunk_size,
            overlap=args.overlap
        )
        
        print("✅ Ingestion completed successfully!")
        
    except Exception as e:
        print(f"❌ Error during ingestion: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()