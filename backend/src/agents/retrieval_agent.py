from typing import List, Dict, Any, Optional
from src.agents.openai_agent import openai_agent
from src.services.retrieval_service import retrieval_service
from src.models.query import QueryRequest, QueryResponse, RetrievedChunk
from src.models.agent import AgentState
from src.config.constants import ERROR_NO_RELEVANT_CONTENT
import logging
import uuid
from datetime import datetime

logger = logging.getLogger(__name__)


class RetrievalAgent:
    def __init__(self):
        self.openai_agent = openai_agent
        self.retrieval_service = retrieval_service
        self.agent_states: Dict[str, AgentState] = {}

    async def process_query(
        self,
        query_request: QueryRequest
    ) -> QueryResponse:
        """
        Process a user query by retrieving relevant content and generating a response.
        """
        try:
            query_text = query_request.query
            session_id = query_request.session_id or str(uuid.uuid4())

            # Validate query input
            if not query_text or not query_text.strip():
                raise ValueError("Query text cannot be empty")

            if len(query_text) > 1000:  # Set a reasonable limit
                raise ValueError("Query text is too long")

            # Get or create agent state for this session
            agent_state = await self._get_or_create_agent_state(session_id, query_request.user_id)

            # Check if retrieval service is available before processing
            try:
                # Verify Qdrant connection by checking collection existence
                collection_exists = await self.retrieval_service.qdrant_service.verify_collection()
                if not collection_exists:
                    logger.warning("Qdrant collection does not exist - proceeding with empty results")
                    retrieved_chunks = []
                else:
                    # Retrieve relevant chunks from Qdrant
                    retrieved_chunks = await self.retrieval_service.retrieve_with_fallback(
                        query_text=query_text
                    )
            except Exception as retrieval_error:
                logger.error(f"Error during retrieval: {retrieval_error}")
                # Proceed with empty chunks if retrieval fails, allowing the agent to still respond
                retrieved_chunks = []

            if not retrieved_chunks:
                logger.warning(f"No relevant chunks found for query: {query_text[:100]}...")

            # Generate response using the agent
            response_text = await self.openai_agent.generate_response(
                query=query_text,
                retrieved_chunks=retrieved_chunks,
                conversation_history=agent_state.conversation_history
            )

            # Update conversation history
            agent_state.conversation_history.append({"role": "user", "content": query_text})
            agent_state.conversation_history.append({"role": "assistant", "content": response_text})
            agent_state.last_query_time = datetime.now()

            # Calculate confidence based on number and relevance of retrieved chunks
            confidence = None
            if retrieved_chunks:
                avg_similarity = sum(chunk.similarity_score for chunk in retrieved_chunks) / len(retrieved_chunks)
                confidence = min(1.0, avg_similarity * 1.5)  # Boost slightly to account for relevance
            else:
                confidence = 0.0  # Set confidence to 0 when no chunks are retrieved

            # Extract sources from retrieved chunks
            sources = list(set(chunk.source_document for chunk in retrieved_chunks if chunk.source_document))

            # Create response object
            response = QueryResponse(
                id=str(uuid.uuid4()),
                query=query_text,
                response=response_text,
                retrieved_chunks=retrieved_chunks,
                confidence=confidence,
                sources=sources,
                timestamp=datetime.now()
            )

            return response

        except Exception as e:
            logger.error(f"Error processing query: {e}")
            raise

    async def _get_or_create_agent_state(
        self,
        session_id: str,
        user_id: Optional[str] = None
    ) -> AgentState:
        """
        Get existing agent state for a session or create a new one.
        """
        if session_id not in self.agent_states:
            metadata = {"user_id": user_id} if user_id else {}
            self.agent_states[session_id] = AgentState(
                session_id=session_id,
                metadata=metadata
            )

        return self.agent_states[session_id]

    async def clear_session(self, session_id: str):
        """
        Clear the conversation history for a specific session.
        """
        if session_id in self.agent_states:
            del self.agent_states[session_id]

    async def get_session_state(self, session_id: str) -> Optional[AgentState]:
        """
        Get the current state of a session.
        """
        return self.agent_states.get(session_id)


# Global instance
retrieval_agent = RetrievalAgent()