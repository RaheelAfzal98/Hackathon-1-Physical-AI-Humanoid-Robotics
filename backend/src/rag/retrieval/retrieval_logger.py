"""Logging utilities for retrieval operations."""

import logging
from typing import Dict, List
from datetime import datetime
from ..models.query_request import QueryRequest
from ..models.agent_response import AgentResponse


class RetrievalLogger:
    """Logger for retrieval operations to track performance and debugging information."""
    
    def __init__(self, name: str = "retrieval"):
        """Initialize the retrieval logger."""
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.INFO)
        
        # Create handler if not already set up
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
    
    def log_retrieval_start(self, query: str, top_k: int, threshold: float):
        """Log the start of a retrieval operation."""
        self.logger.info(
            f"Starting retrieval for query: '{query[:50]}...' "
            f"with top_k={top_k} and threshold={threshold}"
        )
    
    def log_retrieval_results(self, query: str, results_count: int, avg_similarity: float):
        """Log the results of a retrieval operation."""
        self.logger.info(
            f"Retrieval for query: '{query[:50]}...' "
            f"returned {results_count} results with avg similarity {avg_similarity:.3f}"
        )
    
    def log_retrieval_error(self, query: str, error: Exception):
        """Log an error during a retrieval operation."""
        self.logger.error(
            f"Retrieval failed for query: '{query[:50]}...' "
            f"with error: {str(error)}"
        )
    
    def log_agent_query(self, request: QueryRequest, response: AgentResponse):
        """Log a full agent query and response."""
        self.logger.info(
            f"Query: '{request.query_text[:100]}...' | "
            f"Response length: {len(response.content)} chars | "
            f"Confidence: {response.confidence:.3f} | "
            f"Sources: {len(response.sources)} | "
            f"Query time: {response.query_time_ms}ms"
        )
    
    def log_fallback_triggered(self, query: str, reason: str):
        """Log when a fallback mechanism is triggered."""
        self.logger.warning(
            f"Fallback triggered for query: '{query[:50]}...' "
            f"due to: {reason}"
        )
    
    def log_performance_metrics(self, metrics: Dict[str, float]):
        """Log performance metrics."""
        self.logger.info("Performance metrics:")
        for key, value in metrics.items():
            self.logger.info(f"  {key}: {value}")