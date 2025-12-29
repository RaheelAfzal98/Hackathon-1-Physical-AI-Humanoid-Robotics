"""Constants for the RAG agent system."""

# Agent configuration defaults
DEFAULT_TEMPERATURE: float = 0.7
DEFAULT_GROUNDING_STRICTNESS: float = 0.5
DEFAULT_RETRIEVAL_TOP_K: int = 5
DEFAULT_SIMILARITY_THRESHOLD: float = 0.3
DEFAULT_RESPONSE_FORMAT: str = 'standard'
DEFAULT_FALLBACK_ENABLED: bool = True
DEFAULT_ENABLE_CITATIONS: bool = True

# Validation constraints
MIN_TEMPERATURE: float = 0.0
MAX_TEMPERATURE: float = 1.0
MIN_GROUNDING_STRICTNESS: float = 0.0
MAX_GROUNDING_STRICTNESS: float = 1.0
MIN_RETRIEVAL_TOP_K: int = 1
MAX_RETRIEVAL_TOP_K: int = 20
MIN_SIMILARITY_THRESHOLD: float = 0.0
MAX_SIMILARITY_THRESHOLD: float = 1.0
MAX_QUERY_LENGTH: int = 2000
MAX_CONTENT_LENGTH: int = 4000
MAX_SOURCES_COUNT: int = 10
MAX_CONTENT_PREVIEW_LENGTH: int = 500
MAX_USER_CONTEXT_SIZE: int = 5 * 1024  # 5KB
MAX_RESPONSE_LENGTH: int = 4000

# Session management
SESSION_CLEANUP_INTERVAL_MINUTES: int = 30
SESSION_RETENTION_HOURS: int = 24

# Performance limits
DEFAULT_TIMEOUT_SECONDS: int = 30
MAX_CONCURRENT_REQUESTS: int = 100

# API Response codes
SUCCESS_CODE: int = 200
CREATED_CODE: int = 201
NO_CONTENT_CODE: int = 204
BAD_REQUEST_CODE: int = 400
UNAUTHORIZED_CODE: int = 401
NOT_FOUND_CODE: int = 404
SERVER_ERROR_CODE: int = 500

# Session states
SESSION_STATUS_PENDING: str = 'pending'
SESSION_STATUS_ACTIVE: str = 'active'
SESSION_STATUS_INACTIVE: str = 'inactive'
SESSION_STATUS_ARCHIVED: str = 'archived'

# Response processing states
RESPONSE_STATE_RECEIVED: str = 'received'
RESPONSE_STATE_PROCESSING: str = 'processing'
RESPONSE_STATE_GROUNDED: str = 'grounded'
RESPONSE_STATE_VALIDATED: str = 'validated'
RESPONSE_STATE_DELIVERED: str = 'delivered'

# Model names
DEFAULT_OPENAI_MODEL: str = 'mistralai/devstral-2512:free'
EMBEDDING_MODEL: str = 'embed-english-v3.0'

# Logging
LOGGING_LEVEL: str = 'INFO'
LOGGING_FORMAT: str = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'

# Error messages
ERROR_INVALID_TEMPERATURE: str = 'Temperature must be between 0.0 and 1.0'
ERROR_INVALID_GROUNDING_STRICTNESS: str = 'Grounding strictness must be between 0.0 and 1.0'
ERROR_INVALID_RETRIEVAL_TOP_K: str = 'retrieval_top_k must be between 1 and 20'
ERROR_INVALID_SIMILARITY_THRESHOLD: str = 'Similarity threshold must be between 0.0 and 1.0'
ERROR_INVALID_SOURCE_URL: str = 'Source URL must be a valid URL'
ERROR_INVALID_SCORE_RANGE: str = 'Scores must be between 0.0 and 1.0'

# Performance thresholds
GROUNDING_THRESHOLD: float = 0.5  # Minimum grounding score to consider response valid
CONFIDENCE_THRESHOLD: float = 0.6  # Minimum confidence score to consider response reliable