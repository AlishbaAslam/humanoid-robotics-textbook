import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the backend directory to the path so we can import main
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import main


def test_fetch_all_urls():
    """Test the fetch_all_urls function"""
    # This test would require mocking requests and testing URL discovery
    # For now, just test that the function exists and signature is correct
    assert hasattr(main, 'fetch_all_urls')
    assert callable(main.fetch_all_urls)


def test_get_text_from_url():
    """Test the get_text_from_url function"""
    assert hasattr(main, 'get_text_from_url')
    assert callable(main.get_text_from_url)


def test_split_text():
    """Test the split_text function"""
    assert hasattr(main, 'split_text')
    assert callable(main.split_text)

    # Test with a simple text
    text = "This is a test. It has multiple sentences. This is the third one."
    chunks = main.split_text(text, max_chunk_size=50, overlap=10)

    assert len(chunks) > 0
    assert 'text' in chunks[0]
    assert 'chunk_index' in chunks[0]
    assert 'word_count' in chunks[0]
    assert 'content_hash' in chunks[0]


def test_generate_embeddings():
    """Test the generate_embeddings function"""
    assert hasattr(main, 'generate_embeddings')
    assert callable(main.generate_embeddings)


def test_create_rag_collection():
    """Test the create_rag_collection function"""
    assert hasattr(main, 'create_rag_collection')
    assert callable(main.create_rag_collection)


def test_store_chunks_in_qdrant():
    """Test the store_chunks_in_qdrant function"""
    assert hasattr(main, 'store_chunks_in_qdrant')
    assert callable(main.store_chunks_in_qdrant)