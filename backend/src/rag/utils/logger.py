"""Comprehensive logging for all modules in the RAG system."""

import logging
from logging.handlers import RotatingFileHandler
import os
from datetime import datetime
from typing import Dict, Any


def setup_logger(module_name: str, log_file: str = None, level: int = logging.INFO) -> logging.Logger:
    """Set up a logger for the specified module."""
    logger = logging.getLogger(module_name)
    logger.setLevel(level)
    
    # Prevent adding multiple handlers if logger is already set up
    if logger.handlers:
        return logger
    
    # Create formatters and add them to handlers
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s'
    )
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # File handler (rotating to prevent huge log files)
    if log_file:
        file_handler = RotatingFileHandler(
            log_file, maxBytes=10*1024*1024, backupCount=5  # 10MB files, keep 5 backups
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    else:
        # Default location for logs
        log_dir = os.path.join(os.getcwd(), "logs")
        os.makedirs(log_dir, exist_ok=True)
        
        default_log_file = os.path.join(log_dir, f"{module_name}_{datetime.now().strftime('%Y%m%d')}.log")
        file_handler = RotatingFileHandler(
            default_log_file, maxBytes=10*1024*1024, backupCount=5
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger


def log_api_call(
    logger: logging.Logger, 
    endpoint: str, 
    method: str, 
    params: Dict[str, Any] = None, 
    response_code: int = 200, 
    duration_ms: float = 0
):
    """Log an API call with relevant details."""
    logger.info(
        f"API_CALL - {method} {endpoint} | "
        f"Response: {response_code} | "
        f"Duration: {duration_ms:.2f}ms | "
        f"Params: {params if params else 'None'}"
    )


def log_error(logger: logging.Logger, error: Exception, context: str = ""):
    """Log an error with context and traceback."""
    logger.error(
        f"ERROR in {context} - {type(error).__name__}: {str(error)}", 
        exc_info=True  # Include traceback
    )


def log_performance_metric(
    logger: logging.Logger,
    metric_name: str,
    value: float,
    unit: str = "",
    context: str = ""
):
    """Log a performance metric."""
    logger.info(
        f"PERFORMANCE - {metric_name}: {value} {unit} | Context: {context}"
    )


def log_agent_interaction(
    logger: logging.Logger,
    session_id: str,
    query: str,
    response: str,
    response_time_ms: float,
    confidence: float,
    sources_count: int
):
    """Log an agent interaction with full details."""
    logger.info(
        f"AGENT_INTERACTION - Session: {session_id} | "
        f"Query: {query[:50]}... | "
        f"Response Length: {len(response)} | "
        f"Time: {response_time_ms}ms | "
        f"Confidence: {confidence:.2f} | "
        f"Sources: {sources_count}"
    )


def log_retrieval_operation(
    logger: logging.Logger,
    query: str,
    results_count: int,
    avg_similarity: float,
    duration_ms: float
):
    """Log a retrieval operation."""
    logger.info(
        f"RETRIEVAL - Query: {query[:50]}... | "
        f"Results: {results_count} | "
        f"Avg Similarity: {avg_similarity:.3f} | "
        f"Duration: {duration_ms}ms"
    )


def log_validation_result(
    logger: logging.Logger,
    validation_type: str,
    is_valid: bool,
    details: str = ""
):
    """Log a validation result."""
    status = "PASSED" if is_valid else "FAILED"
    logger.info(f"VALIDATION - {validation_type}: {status} | Details: {details}")