#!/usr/bin/env python
"""
Test script to verify bs4 import works correctly in the virtual environment
"""

try:
    from bs4 import BeautifulSoup
    print("✓ BeautifulSoup import successful")

    # Test with a simple HTML string
    html = "<html><body><p>Hello World</p></body></html>"
    soup = BeautifulSoup(html, 'html.parser')
    print(f"✓ BeautifulSoup parsing successful: {soup.p.text}")

    print("✓ All bs4 functionality working correctly!")

except ImportError as e:
    print(f"✗ Import error: {e}")
    import sys
    print(f"Python path: {sys.path}")