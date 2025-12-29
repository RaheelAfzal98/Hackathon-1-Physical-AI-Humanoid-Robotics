"""
Test script to verify agent initialization with environment variables.
"""
import os
from backend.src.config.settings import settings

def test_settings():
    """Test if the settings are loaded correctly from the .env file."""
    print("Testing environment variable loading...")

    print(f"COHERE_API_KEY loaded: {'Yes' if settings.cohere_api_key else 'No'}")
    print(f"QDRANT_URL loaded: {'Yes' if settings.qdrant_url else 'No'}")
    print(f"QDRANT_API_KEY loaded: {'Yes' if settings.qdrant_api_key else 'No'}")
    print(f"OPENAI_API_KEY loaded: {'Yes' if settings.openai_api_key else 'No'}")
    print(f"GEMINI_API_KEY loaded: {'Yes' if settings.gemini_api_key else 'No'}")
    print(f"SOURCE_BASE_URL loaded: {'Yes' if hasattr(settings, 'source_base_url') and settings.source_base_url else 'No'}")

    print(f"\nOpenAI API Key value: {settings.openai_api_key}")
    print(f"Length of OpenAI API Key: {len(settings.openai_api_key) if settings.openai_api_key else 0}")
    print(f"Gemini API Key value: {settings.gemini_api_key}")
    print(f"Length of Gemini API Key: {len(settings.gemini_api_key) if settings.gemini_api_key else 0}")

    # Check if OpenAI key is a placeholder
    if settings.openai_api_key == "your_openai_api_key_here":
        print("\n[WARN] OpenAI API key is still a placeholder. Please replace it with a real API key.")
    elif settings.openai_api_key and len(settings.openai_api_key) > 20:
        print("\n[OK] OpenAI API key appears to be a valid key format.")
    else:
        print("\n[WARN] OpenAI API key doesn't seem to be set properly.")

    # Check if Gemini key is a placeholder
    if settings.gemini_api_key == "your_gemini_api_key_here":
        print("[WARN] Gemini API key is still a placeholder. Please replace it with a real API key.")
    elif settings.gemini_api_key and len(settings.gemini_api_key) > 20:
        print("[OK] Gemini API key appears to be a valid key format.")
    else:
        print("[WARN] Gemini API key doesn't seem to be set properly.")

def test_openai_agent_initialization():
    """Test if we can initialize the OpenAI agent."""
    print("\nTesting OpenAI agent initialization...")

    try:
        from backend.src.rag.agent.openai_agent import OpenAIAgent
        from backend.src.rag.config.agent_config import AgentConfig

        # Check if we have a valid OpenAI API key
        if settings.openai_api_key and settings.openai_api_key != "your_openai_api_key_here" and len(settings.openai_api_key) > 20:
            agent_config = AgentConfig()
            openai_agent = OpenAIAgent(api_key=settings.openai_api_key, config=agent_config)
            print("[OK] OpenAI agent initialized successfully.")
            return True
        else:
            print("[WARN] Cannot initialize OpenAI agent - no valid API key provided.")
            return False
    except Exception as e:
        print(f"[ERROR] Error initializing OpenAI agent: {e}")
        return False

def test_gemini_agent_initialization():
    """Test if we can initialize the Gemini agent."""
    print("\nTesting Gemini agent initialization...")

    try:
        from backend.src.rag.agent.google_gemini_agent import GoogleGeminiAgent
        from backend.src.rag.config.agent_config import AgentConfig

        # Check if we have a valid Gemini API key
        if settings.gemini_api_key and settings.gemini_api_key != "your_gemini_api_key_here" and len(settings.gemini_api_key) > 20:
            agent_config = AgentConfig()
            gemini_agent = GoogleGeminiAgent(config=agent_config)
            gemini_agent.setup_assistant()
            print("[OK] Gemini agent initialized successfully.")
            return True
        else:
            print("[WARN] Cannot initialize Gemini agent - no valid API key provided.")
            return False
    except Exception as e:
        print(f"[ERROR] Error initializing Gemini agent: {e}")
        return False

def test_simple_agent():
    """Test the simple OpenAI agent."""
    print("\nTesting simple OpenAI agent...")

    try:
        from simple_openai_agent import SimpleChatAgent

        # Create a simple agent instance
        agent = SimpleChatAgent()
        print("[OK] Simple OpenAI agent initialized successfully.")
        print(f"Model being used: {agent.model}")
        return True
    except Exception as e:
        print(f"[ERROR] Error initializing simple OpenAI agent: {e}")
        return False

if __name__ == "__main__":
    print("Agent Configuration Test")
    print("=" * 40)

    test_settings()
    test_openai_agent_initialization()
    test_gemini_agent_initialization()
    test_simple_agent()

    print("\n" + "=" * 40)
    print("Test completed.")

    print("\nTo use the OpenAI agent with a real API key:")
    print("1. Go to https://platform.openai.com/api-keys")
    print("2. Create a new API key")
    print("3. Replace 'your_openai_api_key_here' in your .env file with the real key")
    print("4. Restart your application")

    print("\nTo use the Gemini agent with a real API key:")
    print("1. Go to https://aistudio.google.com/app/apikey")
    print("2. Create a new API key")
    print("3. Replace 'your_gemini_api_key_here' in your .env file with the real key")
    print("4. Restart your application")