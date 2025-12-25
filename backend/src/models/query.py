"""
Query models for the Humanoid Robotics Agent API.

This module defines the Pydantic models for query requests, responses, and related data structures.
"""

from pydantic import BaseModel, Field
from pydantic import ConfigDict  # For the model_config
from typing import Optional, List
from datetime import datetime
import uuid


class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000, description="The user's query about humanoid robotics")
    session_id: Optional[str] = Field(None, description="Session identifier to maintain conversation context")
    user_id: Optional[str] = Field(None, description="User identifier for tracking purposes")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "query": "What are the key components of a humanoid robot's locomotion system?",
                "session_id": "sess_abc123xyz",
                "user_id": "user_12345"
            }
        }
    )


class RetrievedChunk(BaseModel):
    id: str
    content: str = Field(..., min_length=10, max_length=5000)
    source_document: str = Field(..., description="Reference to the original document/chapter")
    page_number: Optional[int] = None
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="How similar this chunk is to the query")
    metadata: Optional[dict] = {}

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "id": "chunk_123",
                "content": "The locomotion system of a humanoid robot consists of multiple components working in coordination...",
                "source_document": "Chapter 3: Locomotion Systems",
                "page_number": 45,
                "similarity_score": 0.92,
                "metadata": {"section": "3.2", "keywords": ["actuators", "gait", "balance"]}
            }
        }
    )


class QueryResponse(BaseModel):
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    query: str
    response: str = Field(..., min_length=10, max_length=10000)
    retrieved_chunks: list[RetrievedChunk] = Field(default=[])
    timestamp: datetime = Field(default_factory=datetime.now)
    confidence: Optional[float] = Field(None, ge=0.0, le=1.0, description="Confidence level in the response")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "id": "resp_abc123",
                "query": "What are the key components of a humanoid robot's locomotion system?",
                "response": "The key components of a humanoid robot's locomotion system include: 1) Actuators for joint movement, 2) Sensors for balance and position feedback, 3) Control algorithms for gait planning, and 4) Power systems for sustained operation.",
                "retrieved_chunks": [
                    {
                        "id": "chunk_123",
                        "content": "The locomotion system of a humanoid robot consists of multiple components working in coordination...",
                        "source_document": "Chapter 3: Locomotion Systems",
                        "page_number": 45,
                        "similarity_score": 0.92,
                        "metadata": {"section": "3.2", "keywords": ["actuators", "gait", "balance"]}
                    }
                ],
                "confidence": 0.85,
                "sources": ["Chapter 3: Locomotion Systems", "Chapter 7: Actuator Design"],
                "timestamp": "2025-12-17T10:30:00Z"
            }
        }
    )


class BatchQueryRequest(BaseModel):
    queries: list[QueryRequest] = Field(..., min_length=1, max_length=10)
    session_id: Optional[str] = Field(None, description="Session identifier for batch processing")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "queries": [
                    {
                        "query": "What are the key components of a humanoid robot?",
                        "user_id": "user_12345"
                    },
                    {
                        "query": "How do actuators work in humanoid robots?",
                        "user_id": "user_12345"
                    }
                ],
                "session_id": "sess_abc123xyz"
            }
        }
    )


class BatchQueryResponse(BaseModel):
    responses: list[QueryResponse]
    session_id: Optional[str] = None

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "responses": [
                    {
                        "id": "resp_abc123",
                        "query": "What are the key components of a humanoid robot?",
                        "response": "The key components include actuators, sensors, control systems, and power systems...",
                        "retrieved_chunks": [
                            {
                                "id": "chunk_123",
                                "content": "The main components of a humanoid robot are...",
                                "source_document": "Chapter 2: Robot Components",
                                "similarity_score": 0.95
                            }
                        ],
                        "timestamp": "2025-12-17T10:30:00Z"
                    }
                ],
                "session_id": "sess_abc123xyz"
            }
        }
    )