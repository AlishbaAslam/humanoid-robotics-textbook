import pytest
import os
from unittest.mock import patch


@pytest.fixture
def mock_env_vars():
    """Mock environment variables for testing"""
    with patch.dict(os.environ, {
        'COHERE_API_KEY': 'test-cohere-key',
        'QDRANT_URL': 'https://test-qdrant-url.com',
        'QDRANT_API_KEY': 'test-qdrant-key',
        'BASE_URL': 'https://test-base-url.com'
    }):
        yield


@pytest.fixture
def sample_text():
    """Sample text for testing"""
    return "This is a sample text for testing purposes. It contains multiple sentences. This is the third sentence. And this is the fourth one."