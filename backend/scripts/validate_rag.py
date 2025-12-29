#!/usr/bin/env python3
"""
RAG Retrieval Pipeline Validation Script

This script validates the end-to-end RAG retrieval pipeline by:
1. Testing connectivity to Qdrant and Cohere services
2. Performing sample queries to validate retrieval functionality
3. Verifying that retrieved content maintains traceability to original sources
"""

import sys
import os
import requests
import json
from typing import Dict, List, Any
from datetime import datetime

# Add src to path to import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.config.settings import settings


def validate_environment() -> bool:
    """Validate that all required environment variables are set."""
    required_vars = [
        'COHERE_API_KEY',
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'QDRANT_COLLECTION_NAME'
    ]

    missing_vars = []
    for var in required_vars:
        if not getattr(settings, var.lower(), None):
            missing_vars.append(var)

    if missing_vars:
        print(f"[ERROR] Missing required environment variables: {', '.join(missing_vars)}")
        return False

    print("[SUCCESS] All required environment variables are set")
    return True


def validate_api_connectivity(base_url: str = "http://localhost:8000") -> Dict[str, Any]:
    """Validate API connectivity by calling the validation endpoint."""
    try:
        response = requests.get(f"{base_url}/rag/validate-pipeline", timeout=10)

        if response.status_code == 200:
            result = response.json()
            print("[SUCCESS] API connectivity test passed")
            return {"status": "pass", "details": result}
        else:
            print(f"[ERROR] API connectivity test failed with status {response.status_code}")
            return {"status": "fail", "details": f"Status code: {response.status_code}"}

    except requests.exceptions.RequestException as e:
        print(f"[ERROR] API connectivity test failed with error: {str(e)}")
        return {"status": "fail", "details": str(e)}


def validate_sample_query(base_url: str = "http://localhost:8000") -> Dict[str, Any]:
    """Perform a sample query to validate retrieval functionality."""
    query_payload = {
        "text": "What is RAG in AI?",
        "top_k": 3,
        "similarity_threshold": 0.5
    }

    try:
        response = requests.post(
            f"{base_url}/rag/retrieve",
            json=query_payload,
            timeout=30
        )

        if response.status_code == 200:
            result = response.json()

            # Validate response structure
            required_fields = ["query_text", "results", "total_chunks_processed", "search_time_ms", "retrieval_parameters"]
            missing_fields = [field for field in required_fields if field not in result]

            if missing_fields:
                return {
                    "status": "fail",
                    "details": f"Missing required fields in response: {missing_fields}"
                }

            # Validate that we got results
            if not isinstance(result["results"], list):
                return {"status": "fail", "details": "Results is not a list"}

            # Validate that each result has required metadata
            for i, res in enumerate(result["results"]):
                if "chunk" not in res:
                    return {"status": "fail", "details": f"Result {i} missing 'chunk' field"}

                chunk = res["chunk"]
                if "metadata" not in chunk:
                    return {"status": "fail", "details": f"Result {i} chunk missing 'metadata' field"}

                required_metadata = ["source_url", "page_title", "document_id"]
                missing_metadata = [field for field in required_metadata if field not in chunk["metadata"]]

                if missing_metadata:
                    return {
                        "status": "fail",
                        "details": f"Result {i} chunk missing required metadata: {missing_metadata}"
                    }

            print(f"[SUCCESS] Sample query test passed - retrieved {len(result['results'])} results")
            return {"status": "pass", "details": f"Retrieved {len(result['results'])} results with complete metadata"}

        else:
            print(f"[ERROR] Sample query test failed with status {response.status_code}")
            return {"status": "fail", "details": f"Status code: {response.status_code}, Response: {response.text}"}

    except requests.exceptions.RequestException as e:
        print(f"[ERROR] Sample query test failed with error: {str(e)}")
        return {"status": "fail", "details": str(e)}


def validate_source_traceability(base_url: str = "http://localhost:8000") -> Dict[str, Any]:
    """Validate that retrieved content can be traced back to original sources."""
    query_payload = {
        "text": "Humanoid robotics",
        "top_k": 2,
        "similarity_threshold": 0.3
    }

    try:
        response = requests.post(
            f"{base_url}/rag/retrieve",
            json=query_payload,
            timeout=30
        )

        if response.status_code == 200:
            result = response.json()

            for i, res in enumerate(result["results"]):
                chunk = res["chunk"]
                metadata = chunk["metadata"]

                # Check that critical traceability fields exist and are not empty
                if not metadata.get("source_url"):
                    return {"status": "fail", "details": f"Result {i} missing or empty source_url"}

                if not metadata.get("document_id"):
                    return {"status": "fail", "details": f"Result {i} missing or empty document_id"}

                if "chunk_index" not in metadata or metadata["chunk_index"] is None:
                    return {"status": "fail", "details": f"Result {i} missing chunk_index"}

            print(f"[SUCCESS] Source traceability test passed for {len(result['results'])} results")
            return {"status": "pass", "details": f"Verified traceability for {len(result['results'])} results"}

        else:
            print(f"[ERROR] Source traceability test failed with status {response.status_code}")
            return {"status": "fail", "details": f"Status code: {response.status_code}"}

    except requests.exceptions.RequestException as e:
        print(f"[ERROR] Source traceability test failed with error: {str(e)}")
        return {"status": "fail", "details": str(e)}


def run_full_validation(base_url: str = "http://localhost:8000") -> bool:
    """Run the complete validation suite."""
    print(f"Starting RAG retrieval pipeline validation at {datetime.now().isoformat()}")
    print(f"Testing against: {base_url}")
    print("-" * 60)

    # Initialize results
    results = []

    # Test 1: Environment validation
    print("\n[INFO] Testing: Environment Configuration")
    env_ok = validate_environment()
    results.append({
        "test_name": "environment_check",
        "status": "pass" if env_ok else "fail",
        "details": "Environment variables are properly set" if env_ok else "Missing required environment variables"
    })

    if not env_ok:
        print("[ERROR] Environment validation failed, stopping tests")
        return False

    # Test 2: API Connectivity
    print("\n[INFO] Testing: API Connectivity")
    api_result = validate_api_connectivity(base_url)
    results.append({
        "test_name": "api_connectivity",
        "status": api_result["status"],
        "details": api_result["details"]
    })

    if api_result["status"] == "fail":
        print("[ERROR] API connectivity failed, stopping tests")
        return False

    # Test 3: Sample Query
    print("\n[INFO] Testing: Sample Query Retrieval")
    query_result = validate_sample_query(base_url)
    results.append({
        "test_name": "sample_query",
        "status": query_result["status"],
        "details": query_result["details"]
    })

    # Test 4: Source Traceability
    print("\n[INFO] Testing: Source Traceability")
    trace_result = validate_source_traceability(base_url)
    results.append({
        "test_name": "source_traceability",
        "status": trace_result["status"],
        "details": trace_result["details"]
    })

    # Summary
    print("\n" + "=" * 60)
    print("VALIDATION SUMMARY")
    print("=" * 60)

    passed_count = sum(1 for r in results if r["status"] == "pass")
    total_count = len(results)

    for result in results:
        status_icon = "[PASS]" if result["status"] == "pass" else "[FAIL]"
        print(f"{status_icon} {result['test_name']}: {result['status'].upper()} - {result['details']}")

    print(f"\nOverall: {passed_count}/{total_count} tests passed")

    if passed_count == total_count:
        print("[SUCCESS] All validation tests passed! The RAG retrieval pipeline is working correctly.")
        return True
    else:
        print("[WARNING] Some validation tests failed. Please review the issues above.")
        return False


if __name__ == "__main__":
    # Get base URL from command line argument or use default
    base_url = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8000"

    success = run_full_validation(base_url)
    sys.exit(0 if success else 1)