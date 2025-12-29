from pydantic_settings import BaseSettings
from typing import Optional
from pydantic import Field
from dotenv import load_dotenv
import os

# Load environment variables from .env file with override
load_dotenv(override=True)

class Settings(BaseSettings):
    # Cohere settings
    cohere_api_key: str
    embedding_model: str = "embed-english-v3.0"  # Updated to match requirements

    # Qdrant settings
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "physical_ai_humanoid_docs"  # Updated to match requirements
    qdrant_port: int = 6333

    # OpenAI settings (optional)
    openai_api_key: Optional[str] = None
    openai_model: str = "gpt-4-turbo-preview"  # Updated to match requirements

    # OpenRouter settings
    openrouter_api_key: Optional[str] = None

    # Google Gemini settings (deprecated - not used in current implementation)
    gemini_api_key: Optional[str] = None

    # Source settings
    source_base_url: str = "https://hackathon-1-physical-ai-humanoid-ro-omega.vercel.app/"

    # Application settings
    app_name: str = "RAG Retrieval API"
    debug: bool = False
    version: str = "1.0.0"

    model_config = {"env_file": ".env", "env_file_encoding": "utf-8", "extra": "ignore"}


# Create a single instance of settings
settings = Settings()