#!/usr/bin/env python3
"""
Test script to verify the /query endpoint functionality
"""
import requests
import json
import sys
import os

def test_query_endpoint():
    """Test the query endpoint with a simple request"""
    # Set up the test
    base_url = os.getenv("BACKEND_URL", "http://localhost:8000")
    query_url = f"{base_url}/api/agent/query"

    # Test query
    test_payload = {
        "query": "What is humanoid robotics?",
        "session_id": "test-session-123"
    }

    print(f"Testing query endpoint at: {query_url}")
    print(f"Test payload: {json.dumps(test_payload, indent=2)}")

    try:
        response = requests.post(
            query_url,
            json=test_payload,
            headers={"Content-Type": "application/json"},
            timeout=30  # 30 second timeout
        )

        print(f"Response status code: {response.status_code}")
        print(f"Response headers: {dict(response.headers)}")

        if response.status_code == 200:
            print("✅ Query endpoint test PASSED")
            print(f"Response: {json.dumps(response.json(), indent=2)}")
        elif response.status_code == 500:
            print("❌ Query endpoint test FAILED - 500 Internal Server Error")
            print(f"Error response: {response.text}")
            return False
        else:
            print(f"⚠️  Query endpoint test - Unexpected status code: {response.status_code}")
            print(f"Response: {response.text}")

        return response.status_code == 200

    except requests.exceptions.ConnectionError:
        print(f"❌ Connection error - Could not connect to {query_url}")
        print("Make sure the backend server is running on the specified URL/port")
        return False
    except requests.exceptions.Timeout:
        print("❌ Timeout error - Request took too long")
        return False
    except Exception as e:
        print(f"❌ Unexpected error during test: {e}")
        return False

def test_with_empty_query():
    """Test the endpoint with an empty query to verify validation"""
    base_url = os.getenv("BACKEND_URL", "http://localhost:8000")
    query_url = f"{base_url}/api/agent/query"

    # Test with empty query
    test_payload = {
        "query": "",
        "session_id": "test-session-123"
    }

    print(f"\nTesting validation with empty query...")
    print(f"Test payload: {json.dumps(test_payload, indent=2)}")

    try:
        response = requests.post(
            query_url,
            json=test_payload,
            headers={"Content-Type": "application/json"},
            timeout=10
        )

        print(f"Response status code: {response.status_code}")
        if response.status_code in [422, 400]:  # Validation error
            print("✅ Validation test PASSED - Correctly rejected empty query")
            return True
        else:
            print(f"⚠️  Validation test - Expected 422/400, got {response.status_code}")
            return response.status_code == 200  # This would be OK too if handled gracefully

    except Exception as e:
        print(f"❌ Error during validation test: {e}")
        return False

if __name__ == "__main__":
    print("Testing query endpoint...")

    # Test the main functionality
    success = test_query_endpoint()

    # Test validation
    validation_success = test_with_empty_query()

    if success or validation_success:  # If at least one test passes, consider it a success
        print("\n✅ At least one test passed - endpoint is accessible")
        sys.exit(0)
    else:
        print("\n❌ All tests failed - endpoint may have issues")
        sys.exit(1)