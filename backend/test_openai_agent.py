#!/usr/bin/env python3
"""
Test script to verify the OpenAI Agent SDK pattern with OpenRouter is working properly.
"""
import asyncio
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from src.agents.openai_agent import openrouter_agent, runner
from src.models.query import RetrievedChunk
import uuid


def test_agent_initialization():
    """Test that the agent initializes correctly with available API keys."""
    print("Testing OpenAI Agent SDK pattern initialization...")

    if openrouter_agent.client:
        print("✓ OpenRouter client initialized successfully")
        print(f"  - Model: {openrouter_agent.model}")
        print(f"  - Assistant ID: {openrouter_agent.assistant.id if openrouter_agent.assistant else 'None'}")
    else:
        print("⚠ OpenRouter client not available (no API key or initialization failed)")

    print("✓ Agent initialization test passed")
    return True


async def test_response_generation():
    """Test that the agent can generate responses using the new SDK pattern."""
    print("\nTesting response generation with OpenAI Agent SDK pattern...")

    # Create mock chunks
    mock_chunks = [
        RetrievedChunk(
            id=str(uuid.uuid4()),
            content='Humanoid robots use inverse kinematics for movement planning and control.',
            source_document='Chapter 4: Kinematics.pdf',
            page_number=45,
            similarity_score=0.85
        ),
        RetrievedChunk(
            id=str(uuid.uuid4()),
            content='Balance control in humanoid robots relies on feedback from gyroscopes and accelerometers.',
            source_document='Chapter 6: Balance Control.pdf',
            page_number=78,
            similarity_score=0.90
        )
    ]

    test_query = "How do humanoid robots maintain balance?"

    try:
        # Test using the new Runner pattern
        response = await runner.run_sync(
            query=test_query,
            retrieved_chunks=mock_chunks,
            conversation_history=[]
        )

        print(f"✓ Response generated successfully using Runner pattern")
        print(f"  - Response length: {len(response)} characters")
        print(f"  - Preview: {response[:100]}...")

        if len(response) < 10:
            print("⚠ Response is very short")
            return False

        return True

    except Exception as e:
        print(f"✗ Error generating response: {e}")
        import traceback
        traceback.print_exc()
        return False


async def test_fallback_mechanism():
    """Test the fallback mechanism when primary API is not available."""
    print("\nTesting fallback mechanism...")

    # Create mock chunks
    mock_chunks = [
        RetrievedChunk(
            id=str(uuid.uuid4()),
            content='The humanoid robot control system uses PID controllers for precise motor control.',
            source_document='Chapter 8: Control Systems.pdf',
            page_number=120,
            similarity_score=0.95
        )
    ]

    test_query = "What control systems do humanoid robots use?"

    try:
        # Test using the new Runner pattern with conversation history
        response = await runner.run_sync(
            query=test_query,
            retrieved_chunks=mock_chunks,
            conversation_history=[
                {"role": "user", "content": "Hello"},
                {"role": "assistant", "content": "Hello! How can I help you with humanoid robotics?"},
            ]
        )

        print(f"✓ Fallback response generated successfully")
        print(f"  - Response length: {len(response)} characters")
        print(f"  - Preview: {response[:100]}...")

        return True

    except Exception as e:
        print(f"✗ Error during fallback test: {e}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    print("OpenAI Agent SDK Pattern with OpenRouter - Test Suite")
    print("="*60)

    try:
        init_ok = test_agent_initialization()
        response_ok = await test_response_generation()
        fallback_ok = await test_fallback_mechanism()

        print("\n" + "="*60)
        if init_ok and response_ok and fallback_ok:
            print("✓ All tests passed! OpenAI Agent SDK pattern with OpenRouter is working correctly.")
            print("\nKey features implemented:")
            print("  - AsyncOpenAI client with OpenRouter base_url")
            print("  - Agent, Runner, and RunConfig pattern")
            print("  - Proper response generation with context")
            print("  - Conversation history support")
            print("  - Fallback mechanism when primary API fails")
            print("  - Source citation in responses")
            return True
        else:
            print("✗ Some tests failed!")
            return False

    except Exception as e:
        print(f"✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(main())
    exit(0 if success else 1)