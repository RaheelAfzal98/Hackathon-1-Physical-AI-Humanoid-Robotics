"""
Script to list available Google Gemini models.
"""
import google.generativeai as genai
from backend.src.config.settings import settings

def list_available_models():
    """List all available models from the Google Generative AI API."""
    try:
        # Configure the API with the key
        genai.configure(api_key=settings.gemini_api_key)
        
        # List all models
        print("Available models:")
        for model in genai.list_models():
            print(f"- {model.name}")
            print(f"  - Description: {model.description}")
            print(f"  - Supported generation methods: {model.supported_generation_methods}")
            print()
        
        return True
    except Exception as e:
        print(f"Error listing models: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Listing available Google Gemini models...")
    print("=" * 50)
    list_available_models()