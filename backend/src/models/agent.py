"""
Agent models for the Humanoid Robotics Agent API.

This module defines the Pydantic models for agent configuration and state management.
"""

from pydantic import BaseModel, Field
from pydantic import ConfigDict  # For the model_config
from typing import Optional, Dict, Any, List
from datetime import datetime
import uuid


class AgentConfig(BaseModel):
    model: str = Field(default="mistralai/devstral-2512", description="The model to use (supports OpenRouter models)")
    temperature: float = Field(default=0.7, ge=0.0, le=1.0, description="Sampling temperature")
    max_tokens: Optional[int] = Field(default=1000, description="Maximum tokens in response")
    top_p: float = Field(default=1.0, ge=0.0, le=1.0, description="Top-p sampling parameter")
    frequency_penalty: float = Field(default=0.0, ge=-2.0, le=2.0)
    presence_penalty: float = Field(default=0.0, ge=-2.0, le=2.0)
    # Additional parameters for OpenRouter compatibility
    provider: Optional[str] = Field(default="openrouter", description="The provider to use")
    base_url: Optional[str] = Field(default="https://openrouter.ai/api/v1", description="Base URL for the API provider")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "model": "mistralai/devstral-2512",
                "temperature": 0.7,
                "max_tokens": 1000,
                "top_p": 1.0,
                "frequency_penalty": 0.0,
                "presence_penalty": 0.0,
                "provider": "openrouter",
                "base_url": "https://openrouter.ai/api/v1"
            }
        }
    )


class AgentState(BaseModel):
    session_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    conversation_history: List[Dict[str, str]] = Field(default=[])
    last_query_time: datetime = Field(default_factory=datetime.now)
    config: AgentConfig = Field(default_factory=AgentConfig)
    metadata: Dict[str, Any] = Field(default={})

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "session_id": "sess_abc123xyz",
                "conversation_history": [
                    {
                        "role": "user",
                        "content": "What are the key components of a humanoid robot?"
                    },
                    {
                        "role": "assistant",
                        "content": "The key components include actuators, sensors, control systems..."
                    }
                ],
                "last_query_time": "2025-12-17T10:30:00Z",
                "config": {
                    "model": "mistralai/devstral-2512",
                    "temperature": 0.7,
                    "provider": "openrouter",
                    "base_url": "https://openrouter.ai/api/v1"
                },
                "metadata": {
                    "user_id": "user_12345",
                    "query_count": 1
                }
            }
        }
    )