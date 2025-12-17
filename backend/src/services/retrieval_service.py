from typing import List, Optional
from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service
from src.models.query import RetrievedChunk
from src.config.settings import settings
from src.config.constants import ERROR_NO_RELEVANT_CONTENT, ERROR_QDRANT_CONNECTION
import logging
import time

logger = logging.getLogger(__name__)


class RetrievalService:
    def __init__(self):
        self.qdrant_service = qdrant_service
        self.embedding_service = embedding_service

    async def retrieve_relevant_chunks(
        self,
        query_text: str,
        top_k: Optional[int] = None,
        similarity_threshold: Optional[float] = None
    ) -> List[RetrievedChunk]:
        """
        Main retrieval function that accepts query text and returns relevant chunks from Qdrant.
        """
        start_time = time.time()

        try:
            # Use default settings if not provided
            top_k = top_k or settings.top_k
            similarity_threshold = similarity_threshold or settings.similarity_threshold

            # Validate inputs
            if not query_text or not query_text.strip():
                raise ValueError("Query text cannot be empty")

            # Validate query length
            if len(query_text) > settings.max_query_length:
                raise ValueError(f"Query text exceeds maximum length of {settings.max_query_length} characters")

            # Verify collection exists
            if not await self.qdrant_service.verify_collection():
                raise ValueError(f"Collection '{self.qdrant_service.collection_name}' does not exist in Qdrant")

            # Generate embedding for the query
            query_embedding = await self.embedding_service.generate_query_embedding(query_text)

            # Perform similarity search
            search_results = await self.qdrant_service.search_collection(
                query_vector=query_embedding,
                top_k=top_k,
                similarity_threshold=similarity_threshold
            )

            execution_time = time.time() - start_time
            logger.info(f"Retrieved {len(search_results)} chunks for query in {execution_time:.4f}s")

            return search_results

        except Exception as e:
            execution_time = time.time() - start_time
            logger.error(f"Error in retrieve_relevant_chunks: {e}")
            raise

    async def retrieve_with_fallback(
        self,
        query_text: str,
        top_k: Optional[int] = None,
        similarity_threshold: Optional[float] = None
    ) -> List[RetrievedChunk]:
        """
        Retrieve chunks with fallback logic for different similarity thresholds.
        """
        try:
            # Try with the specified threshold
            results = await self.retrieve_relevant_chunks(
                query_text=query_text,
                top_k=top_k,
                similarity_threshold=similarity_threshold
            )

            # If no results found, try with a lower threshold as fallback
            if not results:
                logger.info("No results found with specified threshold, trying with lower threshold")
                fallback_threshold = (similarity_threshold or settings.similarity_threshold) * 0.7
                results = await self.retrieve_relevant_chunks(
                    query_text=query_text,
                    top_k=top_k,
                    similarity_threshold=max(0.1, fallback_threshold)
                )

            return results
        except Exception as e:
            logger.error(f"Error in retrieve_with_fallback: {e}")
            raise


# Global instance
retrieval_service = RetrievalService()