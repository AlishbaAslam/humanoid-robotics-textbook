#!/usr/bin/env python3
"""
Test script to verify the GEMINI_MODEL configuration is working properly.
"""
import os
import sys
from pathlib import Path

# Add backend to Python path
sys.path.insert(0, str(Path(__file__).parent))

from src.config.settings import settings
from src.agents.gemini_agent import gemini_agent

def test_model_config():
    print("Testing Gemini model configuration...")
    print(f"Settings agent_model: {settings.agent_model}")
    print(f"Settings GEMINI_MODEL: {settings.GEMINI_MODEL}")

    # Determine which model would be used
    model_to_use = settings.GEMINI_MODEL if settings.GEMINI_MODEL else settings.agent_model
    print(f"Model that would be used: {model_to_use}")

    # Check if we have a valid model configured
    if model_to_use:
        print(f"✓ Configuration is correct: {model_to_use} will be used")
        return True
    else:
        print(f"✗ Configuration issue: no model configured")
        return False

def test_agent_initialization():
    print("\nTesting agent initialization...")
    try:
        # Check if the global gemini_agent instance has been created
        if hasattr(gemini_agent, 'model'):
            print(f"✓ Agent initialized successfully with model: {gemini_agent.model}")
            return True
        else:
            print("✗ Agent not properly initialized")
            return False
    except Exception as e:
        print(f"✗ Agent initialization failed: {e}")
        return False

if __name__ == "__main__":
    print("Gemini Model Configuration Test")
    print("="*50)

    config_ok = test_model_config()
    agent_ok = test_agent_initialization()

    print("\n" + "="*50)
    if config_ok and agent_ok:
        print("✓ All tests passed!")
        sys.exit(0)
    else:
        print("✗ Some tests failed!")
        sys.exit(1)