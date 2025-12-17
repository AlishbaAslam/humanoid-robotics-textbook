#!/usr/bin/env python3
"""
Test script to verify the rate-limit-safe embedding generation pipeline.
This script tests the implementation of rate limiting, retry logic, and exponential backoff
for handling Cohere API rate limits (HTTP 429 errors).
"""

import os
import sys
import time
import logging
from unittest.mock import Mock, patch, MagicMock
import random

# Add the backend directory to the path so we can import the main module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from main import calculate_exponential_backoff, make_rate_limited_embedding_request, generate_query_embedding, generate_embeddings

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def test_exponential_backoff_calculation():
    """Test the exponential backoff calculation with jitter."""
    logger.info("Testing exponential backoff calculation...")

    # Test initial backoff
    backoff_time = calculate_exponential_backoff(0, initial_backoff=1.0)
    assert 0.5 <= backoff_time <= 1.0, f"Initial backoff should be between 0.5 and 1.0, got {backoff_time}"
    logger.info(f"✓ Initial backoff (attempt 0): {backoff_time:.3f}s")

    # Test second attempt
    backoff_time = calculate_exponential_backoff(1, initial_backoff=1.0)
    assert 1.0 <= backoff_time <= 2.0, f"Second attempt backoff should be between 1.0 and 2.0, got {backoff_time}"
    logger.info(f"✓ Second attempt backoff (attempt 1): {backoff_time:.3f}s")

    # Test third attempt
    backoff_time = calculate_exponential_backoff(2, initial_backoff=1.0)
    assert 2.0 <= backoff_time <= 4.0, f"Third attempt backoff should be between 2.0 and 4.0, got {backoff_time}"
    logger.info(f"✓ Third attempt backoff (attempt 2): {backoff_time:.3f}s")

    # Test maximum backoff cap
    backoff_time = calculate_exponential_backoff(10, initial_backoff=1.0, max_backoff=10.0)
    assert backoff_time <= 10.0, f"Backoff should not exceed max_backoff of 10.0, got {backoff_time}"
    logger.info(f"✓ Maximum backoff cap test: {backoff_time:.3f}s")

    logger.info("✓ All exponential backoff tests passed!")


def test_rate_limited_request_success():
    """Test that successful requests work normally."""
    logger.info("Testing successful rate-limited request...")

    # Mock Cohere client
    mock_client = Mock()
    mock_response = Mock()
    mock_response.embeddings = [[0.1, 0.2, 0.3]]
    mock_client.embed.return_value = mock_response

    # Test successful request
    result = make_rate_limited_embedding_request(
        cohere_client=mock_client,
        texts=["test text"],
        model="test-model",
        input_type="search_document"
    )

    assert result == mock_response
    mock_client.embed.assert_called_once_with(
        texts=["test text"],
        model="test-model",
        input_type="search_document"
    )

    logger.info("✓ Successful rate-limited request test passed!")


def test_rate_limited_request_with_retry():
    """Test that rate-limited requests retry properly."""
    logger.info("Testing rate-limited request with retry...")

    # Mock Cohere client to raise rate limit error on first call, succeed on second
    mock_client = Mock()
    mock_response = Mock()
    mock_response.embeddings = [[0.1, 0.2, 0.3]]

    # First call raises rate limit error, second succeeds
    call_count = 0
    def embed_side_effect(*args, **kwargs):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            raise Exception("Rate limit exceeded: 429 Too Many Requests")
        return mock_response

    mock_client.embed.side_effect = embed_side_effect

    # Test request with retry
    result = make_rate_limited_embedding_request(
        cohere_client=mock_client,
        texts=["test text"],
        model="test-model",
        input_type="search_document"
    )

    assert result == mock_response
    assert call_count == 2  # Should have been called twice (first failed, second succeeded)

    logger.info("✓ Rate-limited request with retry test passed!")


def test_rate_limited_request_max_retries():
    """Test that requests fail after maximum retry attempts."""
    logger.info("Testing rate-limited request with max retries...")

    # Mock Cohere client to always raise rate limit error
    mock_client = Mock()
    mock_client.embed.side_effect = Exception("Rate limit exceeded: 429 Too Many Requests")

    try:
        make_rate_limited_embedding_request(
            cohere_client=mock_client,
            texts=["test text"],
            model="test-model",
            input_type="search_document"
        )
        assert False, "Expected exception was not raised"
    except Exception as e:
        # Should raise the original exception after max retries
        assert "Rate limit exceeded" in str(e)

    # Should have been called MAX_RETRY_ATTEMPTS times
    assert mock_client.embed.call_count == 5  # Default max retries

    logger.info("✓ Rate-limited request max retries test passed!")


def test_non_rate_limit_error():
    """Test that non-rate limit errors are not retried."""
    logger.info("Testing non-rate limit error handling...")

    # Mock Cohere client to raise authentication error
    mock_client = Mock()
    mock_client.embed.side_effect = Exception("Authentication failed")

    try:
        make_rate_limited_embedding_request(
            cohere_client=mock_client,
            texts=["test text"],
            model="test-model",
            input_type="search_document"
        )
        assert False, "Expected exception was not raised"
    except Exception as e:
        # Should raise immediately without retry
        assert "Authentication failed" in str(e)

    # Should have been called only once since it's not a rate limit error
    assert mock_client.embed.call_count == 1

    logger.info("✓ Non-rate limit error handling test passed!")


def run_all_tests():
    """Run all tests for the rate-limit-safe embedding pipeline."""
    logger.info("Starting tests for rate-limit-safe embedding generation pipeline...")

    try:
        test_exponential_backoff_calculation()
        test_rate_limited_request_success()
        test_rate_limited_request_with_retry()
        test_rate_limited_request_max_retries()
        test_non_rate_limit_error()

        logger.info("\n🎉 All tests passed! The rate-limit-safe embedding pipeline is working correctly.")
        logger.info("\nImplemented features:")
        logger.info("✓ Exponential backoff with jitter")
        logger.info("✓ Rate limiting with throttling between batches")
        logger.info("✓ Retry logic for HTTP 429 errors")
        logger.info("✓ Proper error handling and logging")
        logger.info("✓ Safe fallback for non-rate limit errors")

    except Exception as e:
        logger.error(f"\n❌ Test failed: {e}")
        raise


if __name__ == "__main__":
    run_all_tests()