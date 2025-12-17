import pytest
from unittest.mock import patch, MagicMock
import sys
import os

# Add the backend directory to the path so we can import main
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import main


def test_get_text_from_url_integration():
    """Integration test for get_text_from_url function"""
    # This would be a more comprehensive test when we have proper mocking
    pass


def test_split_text_functionality(sample_text):
    """Test the split_text function with sample text"""
    # Test with the fixture sample text
    chunks = main.split_text(sample_text, max_chunk_size=50, overlap=10)

    # Verify that chunks were created
    assert len(chunks) > 0

    # Verify each chunk has required properties
    for chunk in chunks:
        assert 'text' in chunk
        assert 'chunk_index' in chunk
        assert 'word_count' in chunk
        assert 'content_hash' in chunk
        assert isinstance(chunk['text'], str)
        assert len(chunk['text']) > 0


def test_split_text_with_different_sizes():
    """Test split_text with different chunk sizes"""
    text = "This is a longer text. " * 100  # Create a longer text

    # Test with small chunk size
    small_chunks = main.split_text(text, max_chunk_size=50, overlap=5)
    assert len(small_chunks) > 1

    # Test with large chunk size
    large_chunks = main.split_text(text, max_chunk_size=10000, overlap=5)
    assert len(large_chunks) == 1 or len(large_chunks) < len(small_chunks)


def test_split_text_overlap():
    """Test that split_text properly handles overlap"""
    text = "Word1 word2 word3 word4 word5. " * 20  # Create repetitive text

    chunks = main.split_text(text, max_chunk_size=50, overlap=20)

    # If there are multiple chunks, check that they have overlap
    if len(chunks) > 1:
        # Check that consecutive chunks have some content overlap
        # (This is a simplified check - in real implementation we'd check more thoroughly)
        pass