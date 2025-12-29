from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import Dict, Any
import logging
from src.rag.models.query_request import QueryRequest
from src.rag.models.agent_response import AgentResponse as RetrievalResponse
from src.rag.models.source_reference import SourceReference as SourceReferenceModel
from src.rag.models import ValidationError
from src.rag.models import PipelineValidationResponse
from src.rag.services.retrieval_service import RetrievalService
from src.rag.ingestion import IngestionPipeline
from src.config.settings import settings
from datetime import datetime


router = APIRouter()
retrieval_service = RetrievalService()
logger = logging.getLogger(__name__)


@router.post("/retrieve", response_model=RetrievalResponse)
async def retrieve_content(query_request: QueryRequest):
    """
    Retrieve semantically relevant content chunks based on a natural language query.

    This endpoint accepts a natural-language query and returns the most semantically
    relevant content chunks with complete metadata (source URL, page title, section, chunk index).
    """
    try:
        # Perform the retrieval
        result = retrieval_service.retrieve(query_request)
        return result
    except ValueError as ve:
        logger.warning(f"Validation error in retrieve_content: {str(ve)}")
        raise HTTPException(status_code=422, detail=f"Validation error: {str(ve)}")
    except Exception as e:
        logger.error(f"Error in retrieve_content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/validate-pipeline", response_model=PipelineValidationResponse)
async def validate_pipeline():
    """
    Validate the end-to-end retrieval pipeline.
    """
    try:
        # Perform validation checks
        validation_results = []

        # Check 1: Qdrant connection
        qdrant_connected = retrieval_service.validate_connection()
        validation_results.append({
            "test_name": "qdrant_connectivity",
            "status": "pass" if qdrant_connected else "fail",
            "details": "Successfully connected to Qdrant" if qdrant_connected else "Failed to connect to Qdrant"
        })

        # Check 2: Cohere availability could be added here

        # Determine overall status
        overall_status = "success" if all(test["status"] == "pass" for test in validation_results) else "failure"

        message = "Pipeline validation successful" if overall_status == "success" else "Some validation checks failed"

        return PipelineValidationResponse(
            status=overall_status,
            message=message,
            results=validation_results
        )
    except Exception as e:
        logger.error(f"Error in validate_pipeline: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/ingest", status_code=202)
async def start_ingestion(background_tasks: BackgroundTasks, urls: list[str]):
    """
    Start the ingestion pipeline to crawl websites, generate embeddings, and store in Qdrant.
    This runs as a background task and returns immediately.
    """
    try:
        # Create ingestion pipeline
        ingestion_pipeline = IngestionPipeline(
            cohere_api_key=settings.cohere_api_key,
            qdrant_url=settings.qdrant_url,
            qdrant_api_key=settings.qdrant_api_key,
            collection_name=settings.qdrant_collection_name
        )

        # Run ingestion in background
        # Note: In a real implementation, you'd want to properly handle background tasks
        # For now, this is illustrative
        import threading
        def run_ingestion():
            try:
                ingestion_pipeline.run_ingestion(urls)  # Fixed variable name
                logger.info("Ingestion pipeline completed successfully")
            except Exception as e:
                logger.error(f"Error in ingestion pipeline: {str(e)}")

        thread = threading.Thread(target=run_ingestion)
        thread.start()

        return {
            "message": "Ingestion started successfully",
            "urls": urls,
            "status": "processing"
        }
    except Exception as e:
        logger.error(f"Error starting ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/ingest/status")
async def get_ingestion_status():
    """
    Get the status of the ingestion pipeline.
    Note: This is a simplified implementation.
    A full implementation would track actual ingestion status.
    """
    return {
        "status": "not_applicable",
        "message": "Status tracking not implemented in this basic version"
    }