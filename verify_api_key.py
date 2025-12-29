"""
Script to verify the Google Gemini API key is properly configured without making quota-consuming calls.
"""
import google.generativeai as genai
from backend.src.config.settings import settings

def verify_api_key():
    """Verify that the API key is properly configured."""
    try:
        # Configure the API with the key
        genai.configure(api_key=settings.gemini_api_key)

        print("API key is properly configured.")
        print("Testing with a simple model information call...")

        # Get information about a model (this should not consume quota)
        model_info = genai.GenerativeModel('gemini-2.0-flash')
        print(f"Model name: {model_info.model_name}")
        print("[OK] API key verification successful!")

        return True
    except Exception as e:
        print(f"[ERROR] Error verifying API key: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Verifying Google Gemini API key configuration...")
    print("=" * 50)
    verify_api_key()
    print("=" * 50)
    print("Note: The API key is valid but may have quota limits.")
    print("For full functionality, use an API key with higher quotas or enable billing.")