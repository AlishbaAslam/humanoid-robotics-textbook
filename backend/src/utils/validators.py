from typing import Optional
from src.models.query import QueryRequest
import re


def validate_query_request(query_request: QueryRequest) -> bool:
    """
    Validate a query request according to business rules.
    """
    # Check query length
    if not (1 <= len(query_request.query) <= 1000):
        raise ValueError("Query must be between 1 and 1000 characters")

    # Check for potentially harmful content (basic sanitization)
    if contains_potential_injection(query_request.query):
        raise ValueError("Query contains potentially harmful content")

    # Validate session_id if provided
    if query_request.session_id:
        if not is_valid_session_id(query_request.session_id):
            raise ValueError("Invalid session ID format")

    # Validate user_id if provided
    if query_request.user_id:
        if not is_valid_user_id(query_request.user_id):
            raise ValueError("Invalid user ID format")

    return True


def contains_potential_injection(text: str) -> bool:
    """
    Check if text contains potential injection patterns.
    """
    injection_patterns = [
        r'<script',  # XSS attempts
        r'javascript:',  # JavaScript URLs
        r'vbscript:',  # VBScript URLs
        r'on\w+\s*=',  # Event handlers
    ]

    text_lower = text.lower()
    for pattern in injection_patterns:
        if re.search(pattern, text_lower):
            return True

    return False


def is_valid_session_id(session_id: str) -> bool:
    """
    Validate session ID format (alphanumeric with hyphens and underscores, 8-64 chars).
    """
    if not (8 <= len(session_id) <= 64):
        return False

    return bool(re.match(r'^[a-zA-Z0-9_-]+$', session_id))


def is_valid_user_id(user_id: str) -> bool:
    """
    Validate user ID format (alphanumeric with hyphens and underscores, 4-32 chars).
    """
    if not (4 <= len(user_id) <= 32):
        return False

    return bool(re.match(r'^[a-zA-Z0-9_-]+$', user_id))


def sanitize_text(text: str) -> str:
    """
    Basic text sanitization to remove potentially harmful content.
    """
    # Remove script tags
    text = re.sub(r'<script[^>]*>.*?</script>', '', text, flags=re.IGNORECASE | re.DOTALL)

    # Remove javascript: and vbscript: URLs
    text = re.sub(r'javascript:', '', text, flags=re.IGNORECASE)
    text = re.sub(r'vbscript:', '', text, flags=re.IGNORECASE)

    # Remove event handlers
    text = re.sub(r'on\w+\s*=', '', text, flags=re.IGNORECASE)

    return text.strip()