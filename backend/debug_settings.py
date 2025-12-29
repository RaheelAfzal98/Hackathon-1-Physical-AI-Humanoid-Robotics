"""Debug script to check settings loading."""

import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

print("Environment variables from os.environ:")
print("QDRANT_URL:", os.environ.get("QDRANT_URL"))
print("QDRANT_API_KEY:", os.environ.get("QDRANT_API_KEY"))
print()

# Now try to load using Pydantic Settings
from src.config.settings import settings

print("Settings loaded from Pydantic Settings:")
print("QDRANT_URL:", settings.qdrant_url)
print("QDRANT_API_KEY:", settings.qdrant_api_key)