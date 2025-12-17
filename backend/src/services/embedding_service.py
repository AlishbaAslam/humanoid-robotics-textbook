from typing import List
from cohere import Client as CohereClient
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)


class EmbeddingService:
    def __init__(self):
        self.client = CohereClient(api_key=settings.cohere_api_key)
        self.model = "embed-multilingual-v3.0"

    async def generate_embeddings(
        self,
        texts: List[str],
        input_type: str = "search_document"
    ) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere API.
        """
        try:
            # Generate embeddings for the texts
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=input_type
            )

            return response.embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise

    async def generate_query_embedding(
        self,
        query_text: str
    ) -> List[float]:
        """
        Generate embedding for a single query text.
        """
        try:
            response = self.client.embed(
                texts=[query_text],
                model=self.model,
                input_type="search_query"  # Using search_query as the input type for queries
            )

            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            logger.error(f"Error generating query embedding: {e}")
            raise


# Global instance
embedding_service = EmbeddingService()