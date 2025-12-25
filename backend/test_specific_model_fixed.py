#!/usr/bin/env python3
"""
Test script to verify the specific model from environment works (with encoding fix)
"""
import os
import sys
from dotenv import load_dotenv
from openai import OpenAI

# Set stdout encoding to handle Unicode properly
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

# Load environment variables
load_dotenv()

# Get the API key and base URL from environment
api_key = os.getenv("OPENROUTER_API_KEY")
base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
model = os.getenv("OPENROUTER_MODEL", "mistralai/devstral-2512")

print(f"Testing the specific model from environment...")
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

    # Test the specific model from environment
    print(f"\nTesting the model '{model}'...")
    response = client.chat.completions.create(
        model=model,
        messages=[{"role": "user", "content": "Hello, test!"}],
        max_tokens=20
    )

    # Safely print the response, handling potential encoding issues
    response_content = response.choices[0].message.content
    try:
        print(f"V Model '{model}' test successful: {response_content[:50]}...")
    except UnicodeEncodeError:
        print(f"V Model '{model}' test successful (response received but has encoding issues)")

    print("\nModel test PASSED")

except Exception as e:
    print(f"X Model test FAILED: {e}")
    import traceback
    traceback.print_exc()