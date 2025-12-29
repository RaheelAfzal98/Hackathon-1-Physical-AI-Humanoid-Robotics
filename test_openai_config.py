"""Test script to verify the OpenAI configuration and agent functionality."""

import asyncio
import os
import sys
from dotenv import load_dotenv

# Add the backend/src directory to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from backend.src.rag.config.openai_config import agent_one


def test_agent():
    """Test the agent configuration and functionality."""
    print("Testing OpenAI Configuration...")
    
    # Load environment variables
    load_dotenv()
    
    # Check if the API key is available
    router_api_key = os.getenv("OPENROUTER_API_KEY")
    if not router_api_key or router_api_key == "":
        print("[ERROR] OPENROUTER_API_KEY not found in environment variables")
        return False
    
    print("[OK] OPENROUTER_API_KEY found in environment")
    
    # Check if the agent is properly configured
    if not agent_one:
        print("[ERROR] Agent not properly initialized")
        return False
    
    print(f"[OK] Agent '{agent_one.name}' initialized successfully")
    print(f"[INFO] Agent instructions: '{agent_one.instructions}'")
    
    return True


async def test_agent_response():
    """Test the agent's response functionality."""
    print("\nTesting agent response functionality...")
    
    try:
        # Test query
        test_query = "Hello, can you tell me about frontend development?"
        
        print(f"Query: {test_query}")
        
        # Get response from the agent
        response = await agent_one.run(test_query)
        
        print(f"Response: {response}")
        
        # Basic validation
        if response and len(response) > 0:
            print("[OK] Agent returned a valid response")
            return True
        else:
            print("[ERROR] Agent returned an empty response")
            return False
            
    except Exception as e:
        print(f"[ERROR] Exception occurred while testing agent: {str(e)}")
        return False


async def main():
    """Main function to run all tests."""
    print("Starting OpenAI Configuration Test...\n")
    
    # Test basic configuration
    config_ok = test_agent()
    
    if config_ok:
        # Test agent functionality
        response_ok = await test_agent_response()
        
        if response_ok:
            print("\n[SUCCESS] All tests passed! The chatbot configuration is working properly.")
        else:
            print("\n[FAILURE] Response test failed. The chatbot may not be working properly.")
    else:
        print("\n[FAILURE] Configuration test failed. Please check your environment variables and configuration.")


if __name__ == "__main__":
    # Run the test
    asyncio.run(main())