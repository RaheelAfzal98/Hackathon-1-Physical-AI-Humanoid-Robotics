"""
Demo script to test the Google Gemini API with the provided API key.
"""
from backend.src.rag.agent.google_gemini_agent import GoogleGeminiAgent
from backend.src.rag.config.agent_config import AgentConfig
from backend.src.rag.models.query_request import QueryRequest


def test_gemini_agent():
    """Test the Google Gemini agent with a sample query."""
    print("Testing Google Gemini Agent with the provided API key...")
    
    try:
        # Create agent configuration
        agent_config = AgentConfig()
        
        # Initialize the Gemini agent
        gemini_agent = GoogleGeminiAgent(config=agent_config)
        gemini_agent.setup_assistant()
        
        print("[OK] Gemini agent initialized successfully.")
        
        # Create a sample query
        query_request = QueryRequest(
            query_text="What is a humanoid robot?",
            session_id="demo_session_1",
            user_context={}
        )
        
        # Run a sample query
        print("\nRunning sample query: 'What is a humanoid robot?'")
        response = gemini_agent.run_query(query_request)
        
        print(f"[OK] Query processed successfully.")
        print(f"Response: {response.content}")
        print(f"Confidence: {response.confidence}")
        print(f"Query time: {response.query_time_ms}ms")
        
        return True
        
    except Exception as e:
        print(f"[ERROR] Error testing Gemini agent: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_with_context():
    """Test the Google Gemini agent with context."""
    print("\n" + "="*50)
    print("Testing Google Gemini Agent with context...")
    
    try:
        # Create agent configuration
        agent_config = AgentConfig()
        
        # Initialize the Gemini agent
        gemini_agent = GoogleGeminiAgent(config=agent_config)
        gemini_agent.setup_assistant()
        
        print("[OK] Gemini agent initialized successfully.")
        
        # Create a sample query
        query_request = QueryRequest(
            query_text="Explain the key features of ROS 2 in robotics?",
            session_id="demo_session_2",
            user_context={}
        )
        
        # Sample context about robotics
        context = """
        ROS (Robot Operating System) is a flexible framework for writing robot software. 
        It is a collection of tools, libraries, and conventions that aim to simplify the 
        task of creating complex and robust robot behavior across a wide variety of 
        robotic platforms. ROS 2 is the next generation of ROS, designed to be suitable 
        for industrial applications and to address the shortcomings of ROS 1.
        """
        
        # Run a sample query with context
        print(f"\nRunning query with context: '{query_request.query_text}'")
        response = gemini_agent.run_query(query_request, context=context)
        
        print(f"[OK] Query with context processed successfully.")
        print(f"Response: {response.content}")
        print(f"Confidence: {response.confidence}")
        print(f"Query time: {response.query_time_ms}ms")
        
        return True
        
    except Exception as e:
        print(f"[ERROR] Error testing Gemini agent with context: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("Google Gemini Agent Demo")
    print("=" * 50)
    
    success1 = test_gemini_agent()
    success2 = test_with_context()
    
    print("\n" + "=" * 50)
    if success1 and success2:
        print("Demo completed successfully! The Gemini agent is working properly.")
    else:
        print("Demo encountered errors. Please check the output above.")
    
    print("\nNote: The Google Generative AI package is deprecated.")
    print("Future updates should migrate to the newer google.genai package when available.")