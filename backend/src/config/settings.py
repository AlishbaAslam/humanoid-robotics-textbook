from pydantic_settings import BaseSettings
from typing import Optional
from pydantic import ConfigDict, Field
from dotenv import load_dotenv
import os


class Settings(BaseSettings):
    # API Keys - making them optional for testing
    cohere_api_key: Optional[str] = None
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "book_embeddings"

    # AI Provider API Keys
    openai_api_key: Optional[str] = None
    gemini_api_key: Optional[str] = None
    openrouter_api_key: Optional[str] = None

    # Application settings
    log_level: str = "INFO"
    host: str = "0.0.0.0"
    port: int = 8000

    # Retrieval settings
    top_k: int = 5
    similarity_threshold: float = 0.5
    max_query_length: int = 1000

    # Agent settings
    agent_model: str = "tngtech/deepseek-r1t2-chimera:free"  # Default to OpenRouter model
    GEMINI_MODEL: str = "gemini-2.5-flash"  # Default Gemini model (kept for compatibility)
    OPENROUTER_MODEL: str = "tngtech/deepseek-r1t2-chimera:free"  # Default OpenRouter model

    # Additional fields that might be in environment
    base_url: Optional[str] = None
    openai_base_url: Optional[str] = None
    openrouter_base_url: Optional[str] = None

    model_config = ConfigDict(extra="ignore")


# Load environment variables from .env file
dotenv_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '.env')
if os.path.exists(dotenv_path):
    load_dotenv(dotenv_path)

settings = Settings()