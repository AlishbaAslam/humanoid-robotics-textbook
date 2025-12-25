"""
Constants module for the Humanoid Robotics Agent API.

This module defines all the constant values used throughout the application,
including enum classes, API response constants, and error messages.
"""

from enum import Enum


class QueryProcessingState(str, Enum):
    """
    Enum representing the possible states of query processing.
    """
    PENDING = "PENDING"
    PROCESSING = "PROCESSING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class AgentRole(str, Enum):
    """
    Enum representing the possible roles in agent conversations.
    """
    SYSTEM = "system"
    USER = "user"
    ASSISTANT = "assistant"


# API Response constants
DEFAULT_TIMEOUT = 30  # seconds
MAX_RETRIEVAL_RESULTS = 10
MIN_RELEVANCE_SCORE = 0.3

# Error messages
ERROR_NO_RELEVANT_CONTENT = "No relevant content found in the database for your query."
ERROR_QDRANT_CONNECTION = "Unable to connect to the vector database. Please try again later."
ERROR_AGENT_PROCESSING = "Error processing your query with the AI agent."