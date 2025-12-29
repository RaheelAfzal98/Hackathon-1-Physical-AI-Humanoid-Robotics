"""Comprehensive validation of the RAG retrieval pipeline."""

import sys
import os
from typing import List, Dict
import logging

# Add the backend src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Load environment variables from .env file if it exists
env_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '.env')
if os.path.exists(env_path):
    with open(env_path, 'r') as file:
        for line in file:
            if line.strip() and not line.startswith('#'):
                key, value = line.strip().split('=', 1)
                os.environ[key] = value

from src.rag.agent.openai_agent import OpenAIAgent
from src.rag.retrieval.qdrant_adapter import QdrantAdapter
from src.rag.agent.grounding_validator import GroundingValidator
from src.rag.services.agent_query_service import AgentQueryService
from src.rag.config.agent_config import AgentConfig


def run_comprehensive_validation():
    """Run comprehensive validation of the RAG retrieval pipeline."""
    
    print("Starting Comprehensive RAG Retrieval Pipeline Validation")
    print("="*60)
    
    # Initialize validation components
    config = AgentConfig()
    qdrant_adapter = QdrantAdapter(config)
    grounding_validator = GroundingValidator()
    openai_agent = OpenAIAgent(config)
    
    agent_query_service = AgentQueryService(
        openai_agent=openai_agent,
        retrieval_tool=qdrant_adapter,
        grounding_validator=grounding_validator
    )
    
    # Test data for validation
    validation_tests = [
        {
            "name": "Basic Content Retrieval",
            "query": "What is the introduction to Physical AI and Humanoid Robotics?",
            "expected_sources": ["intro", "introduction", "overview"],
            "min_expected_results": 1
        },
        {
            "name": "ROS2 Concepts",
            "query": "Explain ROS 2 concepts and how they apply to humanoid robots",
            "expected_sources": ["ROS", "nodes", "topics", "services", "rclpy"],
            "min_expected_results": 2
        },
        {
            "name": "Digital Twin Implementation",
            "query": "How to implement a digital twin for humanoid robotics",
            "expected_sources": ["digital twin", "gazebo", "simulation", "unity"],
            "min_expected_results": 2
        },
        {
            "name": "NVIDIA Isaac Integration",
            "query": "What is NVIDIA Isaac and how does it relate to humanoid robotics?",
            "expected_sources": ["nvidia", "isaac", "ai", "perception", "navigation"],
            "min_expected_results": 2
        }
    ]
    
    results = {
        "total_tests": len(validation_tests),
        "passed_tests": 0,
        "failed_tests": 0,
        "test_details": []
    }
    
    for test_case in validation_tests:
        print(f"\nRunning test: {test_case['name']}")
        print(f"Query: {test_case['query']}")
        
        try:
            # Process the query through the agent service
            from src.rag.models.query_request import QueryRequest
            query_request = QueryRequest(
                query_text=test_case["query"],
                session_id=None,
                user_context={},
                response_options={}
            )
            
            # Get response from agent service
            response = agent_query_service.process_query(query_request)
            
            # Validate response
            is_valid = validate_response(
                response=response,
                expected_sources=test_case["expected_sources"],
                min_expected_results=test_case["min_expected_results"]
            )
            
            test_result = {
                "name": test_case["name"],
                "query": test_case["query"],
                "passed": is_valid,
                "response_length": len(response.content) if response else 0,
                "source_count": len(response.sources) if response and hasattr(response, 'sources') else 0
            }
            
            if is_valid:
                print(f"  Result: PASSED")
                results["passed_tests"] += 1
            else:
                print(f"  Result: FAILED")
                results["failed_tests"] += 1
            
            results["test_details"].append(test_result)
            
        except Exception as e:
            print(f"  Result: ERROR - {str(e)}")
            results["failed_tests"] += 1
            results["test_details"].append({
                "name": test_case["name"],
                "query": test_case["query"], 
                "passed": False,
                "error": str(e)
            })
    
    # Print final summary
    print("\n" + "="*60)
    print("COMPREHENSIVE VALIDATION RESULTS")
    print("="*60)
    print(f"Total Tests: {results['total_tests']}")
    print(f"Passed: {results['passed_tests']}")
    print(f"Failed: {results['failed_tests']}")
    print(f"Success Rate: {results['passed_tests']/results['total_tests']*100:.1f}%")
    
    if results["passed_tests"] == results["total_tests"]:
        print("\n🎉 ALL VALIDATIONS PASSED!")
        print("The RAG retrieval pipeline is functioning correctly.")
    else:
        print(f"\n⚠️  {results['failed_tests']} out of {results['total_tests']} tests failed.")
        print("Review the implementation for issues.")
    
    # Additional validation: check metadata completeness
    print(f"\nValidating metadata completeness...")
    try:
        # Get a sample of stored vectors to check metadata
        sample_results = qdrant_adapter.client.scroll(
            collection_name=config.qdrant_collection_name,
            limit=5
        )
        
        if sample_results[0]:  # If we have results
            sample_point = sample_results[0][0]  # Get first point
            payload = sample_point.payload
            
            required_metadata = ['source_url', 'page_title', 'content']
            missing_fields = [field for field in required_metadata if field not in payload]
            
            if not missing_fields:
                print("✅ All required metadata fields are present in stored vectors")
            else:
                print(f"❌ Missing metadata fields: {missing_fields}")
        else:
            print("⚠️  No vectors found to validate metadata")
    except Exception as e:
        print(f"⚠️  Error during metadata validation: {str(e)}")
    
    print("\nValidation completed!")
    return results["passed_tests"] == results["total_tests"]


def validate_response(response, expected_sources: List[str], min_expected_results: int) -> bool:
    """Validate that the response meets quality criteria."""
    if not response:
        return False
        
    # Check response has content
    if not response.content or len(response.content.strip()) < 10:
        return False
    
    # Check if sources exist and meet minimum count
    if hasattr(response, 'sources'):
        if len(response.sources) < min_expected_results:
            return False
    else:
        # If response doesn't have sources property, check if it has content with source references
        content_has_sources = any(source_term in response.content.lower() for source_term in expected_sources)
        if not content_has_sources:
            return False
    
    # If we have sources, check if they contain expected terms
    if hasattr(response, 'sources') and response.sources:
        source_content = " ".join([str(getattr(source, 'content_preview', '')) for source in response.sources])
        expected_found = any(term.lower() in source_content.lower() for term in expected_sources)
        return expected_found
    else:
        # If no sources, just validate content relevance
        content_lower = response.content.lower()
        expected_found = any(term.lower() in content_lower for term in expected_sources)
        return expected_found


def main():
    """Main function to run the comprehensive validation."""
    success = run_comprehensive_validation()
    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)