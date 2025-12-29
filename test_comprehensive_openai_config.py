"""Comprehensive test script to verify the OpenAI configuration and agent functionality."""

import asyncio
import os
import sys
from dotenv import load_dotenv

# Add the backend/src directory to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from backend.src.rag.config.openai_config import agent_one, provider, model


def test_environment():
    """Test environment variables."""
    print("Testing environment variables...")
    
    # Load environment variables
    load_dotenv()
    
    # Check if the API key is available
    router_api_key = os.getenv("OPENROUTER_API_KEY")
    if not router_api_key or router_api_key == "":
        print("[ERROR] OPENROUTER_API_KEY not found in environment variables")
        return False
    
    print("[OK] OPENROUTER_API_KEY found in environment")
    return True


def test_provider():
    """Test the provider configuration."""
    print("\nTesting provider configuration...")
    
    if not provider:
        print("[ERROR] Provider not initialized")
        return False
    
    print("[OK] Provider initialized successfully")
    return True


def test_model():
    """Test the model configuration."""
    print("\nTesting model configuration...")
    
    if not model:
        print("[ERROR] Model not initialized")
        return False
    
    if not model.model or model.model != "mistralai/devstral-2512:free":
        print(f"[ERROR] Model not configured correctly. Expected 'mistralai/devstral-2512:free', got '{model.model}'")
        return False
    
    print("[OK] Model initialized successfully with correct model name")
    return True


def test_agent():
    """Test the agent configuration."""
    print("\nTesting agent configuration...")
    
    if not agent_one:
        print("[ERROR] Agent not initialized")
        return False
    
    if agent_one.name != "Frontend Expert":
        print(f"[ERROR] Agent name incorrect. Expected 'Frontend Expert', got '{agent_one.name}'")
        return False
    
    if agent_one.instructions != "you are a frontend expert":
        print(f"[ERROR] Agent instructions incorrect. Expected 'you are a frontend expert', got '{agent_one.instructions}'")
        return False
    
    print("[OK] Agent initialized successfully with correct name and instructions")
    return True


async def test_multiple_queries():
    """Test the agent with multiple queries."""
    print("\nTesting agent with multiple queries...")
    
    queries = [
        "What is HTML?",
        "Explain CSS to me.",
        "What does JavaScript do?"
    ]
    
    for i, query in enumerate(queries, 1):
        print(f"\nQuery {i}: {query}")
        try:
            response = await agent_one.run(query)
            print(f"Response {i}: {response[:100]}...")  # Print first 100 chars
            
            if not response or len(response) == 0:
                print(f"[ERROR] Query {i} returned empty response")
                return False
        except Exception as e:
            print(f"[ERROR] Query {i} failed with exception: {str(e)}")
            return False
    
    print("\n[OK] All queries processed successfully")
    return True


async def main():
    """Main function to run all tests."""
    print("Starting Comprehensive OpenAI Configuration Test...\n")
    
    # Run all tests
    env_ok = test_environment()
    provider_ok = test_provider()
    model_ok = test_model()
    agent_ok = test_agent()
    
    if all([env_ok, provider_ok, model_ok, agent_ok]):
        print("\n[OK] All basic configuration tests passed")
        
        # Test functionality
        queries_ok = await test_multiple_queries()
        
        if queries_ok:
            print("\n[SUCCESS] All tests passed! The chatbot configuration is working properly.")
        else:
            print("\n[FAILURE] Query tests failed.")
    else:
        print("\n[FAILURE] Some configuration tests failed.")


if __name__ == "__main__":
    # Run the test
    asyncio.run(main())