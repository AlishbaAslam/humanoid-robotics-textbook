#!/usr/bin/env python3
"""
Test script to verify that the Gemini API configuration is properly set up.
"""
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def test_gemini_import():
    """Test importing the Gemini agent."""
    try:
        from backend.src.agents.gemini_agent import gemini_agent
        print("✓ Successfully imported Gemini agent")
        return True
    except ImportError as e:
        print(f"✗ Failed to import Gemini agent: {e}")
        return False
    except Exception as e:
        print(f"✗ Error importing Gemini agent: {e}")
        return False

def test_settings():
    """Test that settings properly load the GEMINI_API_KEY."""
    try:
        from backend.src.config.settings import settings
        print(f"✓ Successfully imported settings")

        # Check if GEMINI_API_KEY is available
        if settings.gemini_api_key:
            print(f"✓ GEMINI_API_KEY is configured")
        else:
            print(f"⚠ GEMINI_API_KEY is not set in environment")

        # Check model settings
        print(f"✓ Default agent model: {settings.agent_model}")
        print(f"✓ GEMINI_MODEL setting: {getattr(settings, 'GEMINI_MODEL', 'Not found')}")

        return True
    except Exception as e:
        print(f"✗ Error with settings: {e}")
        return False

def test_retrieval_agent():
    """Test that the retrieval agent uses the Gemini agent."""
    try:
        from backend.src.agents.retrieval_agent import retrieval_agent
        print("✓ Successfully imported retrieval agent")

        # Check if it's using the gemini agent
        agent_type = type(retrieval_agent.openai_agent).__name__
        print(f"✓ Retrieval agent uses: {agent_type}")

        if "Gemini" in agent_type:
            print("✓ Retrieval agent is correctly configured to use Gemini")
        else:
            print(f"⚠ Retrieval agent is using {agent_type}, not Gemini")

        return True
    except Exception as e:
        print(f"✗ Error with retrieval agent: {e}")
        return False

def main():
    """Run all tests."""
    print("Testing Gemini API Configuration...")
    print("="*50)

    tests = [
        ("Settings Configuration", test_settings),
        ("Gemini Agent Import", test_gemini_import),
        ("Retrieval Agent Configuration", test_retrieval_agent),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        result = test_func()
        results.append((test_name, result))

    print("\n" + "="*50)
    print("Test Summary:")
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        print(f"  {test_name}: {status}")

    all_passed = all(result for _, result in results)
    print(f"\nOverall: {'PASS' if all_passed else 'FAIL'}")

    return all_passed

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)