from pydantic_settings import BaseSettings
from typing import Optional
from pydantic import ConfigDict, Field


class Settings(BaseSettings):
    # API Keys - making them optional for testing
    openai_api_key: Optional[str] = None
    cohere_api_key: Optional[str] = None
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "book_embeddings"

    # Application settings
    log_level: str = "INFO"
    host: str = "0.0.0.0"
    port: int = 8000

    # Retrieval settings
    top_k: int = 5
    similarity_threshold: float = 0.5
    max_query_length: int = 1000

    # Agent settings
    agent_model: str = "gemini-2.5-flash"
    GEMINI_MODEL: Optional[str] = None

    # Additional fields that might be in environment
    base_url: Optional[str] = None
    gemini_api_key: Optional[str] = None

    model_config = ConfigDict(env_file=".env", extra="ignore")


settings = Settings()