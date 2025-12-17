#!/usr/bin/env python3

"""
Test script to verify bs4 import works correctly
"""

try:
    from bs4 import BeautifulSoup
    print("✓ BeautifulSoup import successful")

    # Test with a simple HTML string
    html = "<html><body><p>Hello World</p></body></html>"
    soup = BeautifulSoup(html, 'html.parser')
    print(f"✓ BeautifulSoup parsing successful: {soup.p.text}")

except ImportError as e:
    print(f"✗ Import error: {e}")

print("✓ bs4 import test completed successfully")