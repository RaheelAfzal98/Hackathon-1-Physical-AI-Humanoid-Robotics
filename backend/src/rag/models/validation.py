"""Validation models for the RAG system."""

from pydantic import BaseModel
from typing import Dict, Any, Optional
from datetime import datetime


class ValidationError(BaseModel):
    error_code: str
    message: str
    details: Optional[Dict[str, Any]] = None
    timestamp: datetime

    class Config:
        json_schema_extra = {
            "example": {
                "error_code": "INVALID_QUERY",
                "message": "The query text is invalid",
                "details": {"field": "query_text", "reason": "too short"},
                "timestamp": "2023-10-01T12:00:00Z"
            }
        }


class PipelineValidationResponse(BaseModel):
    status: str  # "success" or "failure"
    message: str
    results: list[dict]  # List of validation test results

    class Config:
        json_schema_extra = {
            "example": {
                "status": "success",
                "message": "Pipeline validation successful",
                "results": [
                    {
                        "test_name": "qdrant_connectivity",
                        "status": "pass",
                        "details": "Successfully connected to Qdrant"
                    }
                ]
            }
        }