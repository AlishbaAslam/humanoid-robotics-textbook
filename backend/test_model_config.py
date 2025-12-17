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
from src.agents.openai_agent import OpenAIAgent

def test_model_config():
    print("Testing model configuration...")
    print(f"Settings agent_model: {settings.agent_model}")
    print(f"Settings GEMINI_MODEL: {settings.GEMINI_MODEL}")

    # Determine which model would be used
    model_to_use = settings.GEMINI_MODEL if settings.GEMINI_MODEL else settings.agent_model
    print(f"Model that would be used: {model_to_use}")

    if model_to_use == "gemini-2.5-flash":
        print("✓ Configuration is correct: gemini-2.5-flash will be used")
        return True
    else:
        print(f"✗ Configuration issue: expected gemini-2.5-flash, got {model_to_use}")
        return False

def test_agent_initialization():
    print("\nTesting agent initialization...")
    try:
        # This will try to initialize the agent with the configured model
        agent = OpenAIAgent()
        print(f"✓ Agent initialized successfully with model: {agent.model.model_name}")
        return True
    except Exception as e:
        print(f"✗ Agent initialization failed: {e}")
        return False

if __name__ == "__main__":
    print("Model Configuration Test")
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