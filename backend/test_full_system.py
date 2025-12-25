#!/usr/bin/env python3
"""
Test script to verify the full OpenAI Agent SDK pattern with OpenRouter works with retrieval
"""
import os
import asyncio
import uuid
from dotenv import load_dotenv
from src.agents.openai_agent import openrouter_agent, runner
from src.models.query import RetrievedChunk, QueryRequest
from src.agents.retrieval_agent import retrieval_agent

# Load environment variables
load_dotenv()

async def test_full_retrieval_agent():
    print("Testing full OpenAI Agent SDK pattern with OpenRouter...")
    print(f"Using model: {openrouter_agent.model}")

    # Create mock retrieved chunks
    mock_chunks = [
        RetrievedChunk(
            id=str(uuid.uuid4()),
            content='Physical AI refers to the integration of artificial intelligence with physical systems, particularly robots. It involves AI algorithms that control physical actuators and respond to real-world environments.',
            source_document='Chapter 1: Introduction to Physical AI.pdf',
            page_number=12,
            similarity_score=0.85,
            metadata={'section': '1.1', 'keywords': ['physical ai', 'robotics', 'integration']}
        ),
        RetrievedChunk(
            id=str(uuid.uuid4()),
            content='Physical AI systems combine machine learning, computer vision, and robotics to create intelligent machines that can interact with the physical world.',
            source_document='Chapter 2: Physical AI Fundamentals.pdf',
            page_number=25,
            similarity_score=0.80,
            metadata={'section': '2.3', 'keywords': ['machine learning', 'computer vision', 'robotics']}
        )
    ]

    print(f"Created {len(mock_chunks)} mock retrieved chunks")

    try:
        # Test the OpenRouter agent directly first
        print("\nTesting OpenAI Agent SDK pattern directly...")
        response = await openrouter_agent.generate_response(
            query="What is Physical AI?",
            retrieved_chunks=mock_chunks
        )

        print(f"V OpenAI Agent SDK pattern response successful!")
        print(f"Response preview: {response[:100]}...")

        # Test the runner pattern
        print("\nTesting Runner pattern...")
        runner_response = await runner.run_sync(
            query="What is Physical AI?",
            retrieved_chunks=mock_chunks,
            conversation_history=[]
        )
        print(f"V Runner pattern response successful!")
        print(f"Runner response preview: {runner_response[:100]}...")

        # Test the full retrieval agent
        print("\nTesting full retrieval agent...")
        query_request = QueryRequest(
            query="What is Physical AI?",
            session_id="test_session_123"
        )

        full_response = await retrieval_agent.process_query(query_request)
        print(f"V Full retrieval agent response successful!")
        print(f"Full response preview: {full_response.response[:100]}...")
        print(f"Retrieved {len(full_response.retrieved_chunks)} chunks")
        print(f"Confidence: {full_response.confidence}")

        print("\nFull system test PASSED!")
        return True

    except Exception as e:
        print(f"X Full system test FAILED: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(test_full_retrieval_agent())
    exit(0 if success else 1)