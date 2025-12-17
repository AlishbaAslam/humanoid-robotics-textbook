from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
from src.config.settings import settings
from src.models.query import RetrievedChunk
import logging
import uuid

logger = logging.getLogger(__name__)


class QdrantService:
    def __init__(self):
        self._client = None
        self.collection_name = settings.qdrant_collection_name
        self._settings = settings

    @property
    def client(self):
        """Lazy initialization of Qdrant client to avoid startup errors"""
        if self._client is None:
            if not self._settings.qdrant_url:
                raise ValueError("QDRANT_URL environment variable not set")

            # Initialize Qdrant client
            if self._settings.qdrant_api_key:
                self._client = QdrantClient(
                    url=self._settings.qdrant_url,
                    api_key=self._settings.qdrant_api_key,
                    timeout=10
                )
            else:
                self._client = QdrantClient(
                    url=self._settings.qdrant_url,
                    timeout=10
                )
        return self._client

    async def verify_collection(self) -> bool:
        """
        Verify that the specified Qdrant collection exists.
        """
        try:
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            return collection_exists
        except Exception as e:
            logger.error(f"Error verifying Qdrant collection: {e}")
            raise

    async def search_collection(
        self,
        query_vector: List[float],
        top_k: int = 5,
        similarity_threshold: float = 0.5
    ) -> List[RetrievedChunk]:
        """
        Perform vector similarity search in Qdrant collection.
        """
        try:
            # Perform the search in Qdrant
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                score_threshold=similarity_threshold
            )

            # Format the results into RetrievedChunk objects
            retrieved_chunks = []
            for result in search_results.points:
                chunk = RetrievedChunk(
                    id=str(result.id),
                    content=result.payload.get("text", ""),
                    source_document=result.payload.get("source_document", result.payload.get("title", "Unknown")),
                    page_number=result.payload.get("page_number"),
                    similarity_score=result.score,
                    metadata=result.payload.get("metadata", {})
                )
                retrieved_chunks.append(chunk)

            return retrieved_chunks
        except Exception as e:
            logger.error(f"Error performing Qdrant search: {e}")
            raise

    async def store_embeddings(
        self,
        embeddings: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Store vector embeddings in Qdrant with associated metadata.
        """
        try:
            points = []
            for embedding_data in embeddings:
                point = PointStruct(
                    id=str(uuid.uuid4()),  # Using UUID to ensure unique IDs
                    vector=embedding_data["vector"],
                    payload={
                        "text": embedding_data["text"],
                        "source_document": embedding_data.get("source_document", ""),
                        "title": embedding_data.get("title", ""),
                        "page_number": embedding_data.get("page_number"),
                        "metadata": embedding_data.get("metadata", {}),
                        "content_hash": embedding_data.get("content_hash", ""),
                        "chunk_id": embedding_data.get("chunk_id", "")
                    }
                )
                points.append(point)

            # Upload the points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            return {
                "total_processed": len(embeddings),
                "successful": len(embeddings),
                "failed": 0,
                "errors": []
            }
        except Exception as e:
            logger.error(f"Error storing embeddings in Qdrant: {e}")
            raise

    async def create_collection(
        self,
        vector_size: int = 1024,
        distance: str = "Cosine"
    ) -> bool:
        """
        Creates or ensures existence of the Qdrant collection for embeddings.
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create the collection with specified vector size
                distance_enum = models.Distance[distance.upper()]
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=vector_size, distance=distance_enum)
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection already exists: {self.collection_name}")

            return True
        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            raise

    async def count_points(self) -> int:
        """
        Get the total number of points in the collection.
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except Exception as e:
            logger.error(f"Error counting points in Qdrant: {e}")
            raise


# Global instance
qdrant_service = QdrantService()