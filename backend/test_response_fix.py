#!/usr/bin/env python3
"""
Test script to verify that the RAG chatbot always returns valid response strings of at least 10 characters.
"""
import sys
import os
from pathlib import Path

# Add backend to Python path
sys.path.insert(0, str(Path(__file__).parent))

from src.models.query import QueryRequest
from src.agents.retrieval_agent import retrieval_agent

async def test_response_length():
    """Test that responses are always at least 10 characters long."""
    print("Testing response length validation...")

    # Test case 1: Normal query
    print("\n1. Testing normal query...")
    query_request = QueryRequest(
        query="What is a humanoid robot?",
        session_id="test_session_1"
    )

    try:
        response = await retrieval_agent.process_query(query_request)
        print(f"Response length: {len(response.response)}")
        print(f"Response preview: {response.response[:100]}...")
        assert len(response.response) >= 10, f"Response too short: '{response.response}'"
        print("✓ Normal query test passed")
    except Exception as e:
        print(f"✗ Normal query test failed: {e}")

    # Test case 2: Query with no relevant content
    print("\n2. Testing query with no relevant content...")
    query_request = QueryRequest(
        query="This is a completely unrelated query that should not match any content",
        session_id="test_session_2"
    )

    try:
        response = await retrieval_agent.process_query(query_request)
        print(f"Response length: {len(response.response)}")
        print(f"Response preview: {response.response[:100]}...")
        assert len(response.response) >= 10, f"Response too short: '{response.response}'"
        print("✓ No content query test passed")
    except Exception as e:
        print(f"✗ No content query test failed: {e}")

    # Test case 3: Very short potential response
    print("\n3. Testing edge case handling...")
    query_request = QueryRequest(
        query="Tell me about robots",
        session_id="test_session_3"
    )

    try:
        response = await retrieval_agent.process_query(query_request)
        print(f"Response length: {len(response.response)}")
        print(f"Response preview: {response.response[:100]}...")
        assert len(response.response) >= 10, f"Response too short: '{response.response}'"
        print("✓ Edge case test passed")
    except Exception as e:
        print(f"✗ Edge case test failed: {e}")

    print("\nAll tests completed!")

if __name__ == "__main__":
    import asyncio
    asyncio.run(test_response_length())