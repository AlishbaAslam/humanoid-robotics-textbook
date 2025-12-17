from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from datetime import datetime
import uuid


class AgentConfig(BaseModel):
    model: str = Field(default="gpt-4-turbo-preview", description="The OpenAI model to use")
    temperature: float = Field(default=0.7, ge=0.0, le=1.0, description="Sampling temperature")
    max_tokens: Optional[int] = Field(default=1000, description="Maximum tokens in response")
    top_p: float = Field(default=1.0, ge=0.0, le=1.0, description="Top-p sampling parameter")
    frequency_penalty: float = Field(default=0.0, ge=-2.0, le=2.0)
    presence_penalty: float = Field(default=0.0, ge=-2.0, le=2.0)

    class Config:
        json_schema_extra = {
            "example": {
                "model": "gpt-4-turbo-preview",
                "temperature": 0.7,
                "max_tokens": 1000,
                "top_p": 1.0,
                "frequency_penalty": 0.0,
                "presence_penalty": 0.0
            }
        }


class AgentState(BaseModel):
    session_id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    conversation_history: List[Dict[str, str]] = Field(default=[])
    last_query_time: datetime = Field(default_factory=datetime.now)
    config: AgentConfig = Field(default_factory=AgentConfig)
    metadata: Dict[str, Any] = Field(default={})

    class Config:
        json_schema_extra = {
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
                    "model": "gpt-4-turbo-preview",
                    "temperature": 0.7
                },
                "metadata": {
                    "user_id": "user_12345",
                    "query_count": 1
                }
            }
        }