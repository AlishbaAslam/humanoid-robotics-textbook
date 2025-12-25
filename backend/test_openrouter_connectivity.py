#!/usr/bin/env python3
"""
Test script to verify OpenRouter API connectivity and model availability
"""
import os
from dotenv import load_dotenv
from openai import OpenAI

# Load environment variables
load_dotenv()

# Get the API key and base URL from environment
api_key = os.getenv("OPENROUTER_API_KEY")
base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
model = os.getenv("OPENROUTER_MODEL", "mistralai/devstral-2512")

print(f"Testing OpenRouter API connectivity...")
print(f"Base URL: {base_url}")
print(f"Model: {model}")

if not api_key:
    print("ERROR: OPENROUTER_API_KEY not found in environment")
    exit(1)

try:
    # Create OpenAI client with OpenRouter settings
    client = OpenAI(
        api_key=api_key,
        base_url=base_url
    )

    print("V Client created successfully")

    # Test API connectivity by listing models
    print("Testing model listing...")
    models = client.models.list()
    available_models = [model.id for model in models.data]

    print(f"V Successfully connected to OpenRouter API")
    print(f"Available models (first 10): {available_models[:10]}")

    # Check if our model is available
    if model in available_models:
        print(f"V Model '{model}' is available")
    else:
        print(f"X Model '{model}' is NOT available")
        print(f"Available similar models: {[m for m in available_models if 'mistral' in m.lower()][:5]}")

    # Test a simple completion to verify the API key works
    print("\nTesting simple completion...")
    response = client.chat.completions.create(
        model="openchat/openchat-7b:free",  # Use a known working free model
        messages=[{"role": "user", "content": "Hello, world!"}],
        max_tokens=10
    )

    print(f"V Simple completion successful: {response.choices[0].message.content[:50]}...")

    print("\nAPI connectivity test PASSED")

except Exception as e:
    print(f"X API connectivity test FAILED: {e}")
    import traceback
    traceback.print_exc()