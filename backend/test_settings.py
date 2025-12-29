import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from src.config.settings import settings

print("Testing environment variable loading...")

try:
    print(f"Cohere API Key loaded: {'Yes' if settings.cohere_api_key else 'No'}")
    print(f"Qdrant URL loaded: {settings.qdrant_url}")
    print(f"Qdrant API Key loaded: {'Yes' if settings.qdrant_api_key else 'No'}")
    print(f"OpenAI API Key loaded: {'Yes' if settings.openai_api_key else 'No'}")
    print(f"Gemini API Key loaded: {'Yes' if settings.gemini_api_key else 'No'}")
    
    print("\nAll settings loaded successfully!")
    
except Exception as e:
    print(f"Error loading settings: {e}")
    import traceback
    traceback.print_exc()