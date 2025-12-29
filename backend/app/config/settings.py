from dotenv import load_dotenv
import os

# Load environment variables from .env file
load_dotenv()

class Settings:
    # Cohere settings
    cohere_api_key: str = os.getenv("COHERE_API_KEY")

    # Qdrant settings
    qdrant_url: str = os.getenv("QDRANT_URL")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "book_content_chunks_v1")

    # Source settings
    source_base_url: str = os.getenv("SOURCE_BASE_URL", "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/")

    # Processing settings
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "50"))
    batch_size: int = int(os.getenv("BATCH_SIZE", "10"))
    embedding_model: str = os.getenv("EMBEDDING_MODEL", "embed-multilingual-v3.0")
    embedding_dimension: int = int(os.getenv("EMBEDDING_DIMENSION", "1024"))  # For multilingual-v3.0

    # Retry settings
    max_retries: int = int(os.getenv("MAX_RETRIES", "3"))
    delay_base: float = float(os.getenv("DELAY_BASE", "1.0"))
    timeout: int = int(os.getenv("TIMEOUT", "30"))

    # Validation
    def validate(self):
        errors = []
        if not self.cohere_api_key:
            errors.append("COHERE_API_KEY is required")
        if not self.qdrant_url:
            errors.append("QDRANT_URL is required")
        if not self.qdrant_api_key:
            errors.append("QDRANT_API_KEY is required")

        if errors:
            raise ValueError(f"Configuration errors: {'; '.join(errors)}")


settings = Settings()
settings.validate()