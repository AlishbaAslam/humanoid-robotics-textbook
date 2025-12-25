import re
from typing import List, Dict, Any, Optional
from src.agents.openai_agent import openrouter_agent, runner
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
        self.openrouter_agent = openrouter_agent  # Use the new OpenRouter agent
        self.runner = runner  # Use the new runner pattern
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
            retrieved_chunks = []
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

            # Generate response using the new OpenAI Agents SDK pattern
            response_text = await self.runner.run_sync(
                query=query_text,
                retrieved_chunks=retrieved_chunks,
                conversation_history=agent_state.conversation_history
            )

            # Ensure response is at least 10 characters long
            if not response_text or len(response_text) < 10:
                if retrieved_chunks:
                    # If we have retrieved content but the response is too short,
                    # provide a meaningful response based on the retrieved content
                    response_text = "Based on the available information, I found some relevant content but need to provide a more detailed response. " \
                                  "The retrieved information suggests the topic is covered in the textbook, but I recommend checking the specific " \
                                  "source documents for complete details."
                else:
                    # If no retrieved content and response is too short, provide a fallback message
                    response_text = "Sorry, I could not find a relevant answer in the textbook. Please try rephrasing your question or " \
                                  "consult the humanoid robotics textbook directly for more comprehensive information."

            # Final validation to ensure response meets minimum length requirement
            if not response_text or len(response_text) < 10:
                response_text = "I'm sorry, but I couldn't generate a sufficient response. Please try rephrasing your question or " \
                              "consult the humanoid robotics textbook directly for more comprehensive information."

            # Update conversation history
            agent_state.conversation_history.append({"role": "user", "content": query_text})
            agent_state.conversation_history.append({"role": "assistant", "content": response_text})
            agent_state.last_query_time = datetime.now()

            # Calculate confidence based on number and relevance of retrieved chunks
            confidence = 0.0  # Default confidence
            if retrieved_chunks:
                # Calculate average similarity score, avoiding division by zero
                total_similarity = sum(chunk.similarity_score for chunk in retrieved_chunks if chunk.similarity_score is not None)
                avg_similarity = total_similarity / len(retrieved_chunks) if len(retrieved_chunks) > 0 else 0.0
                # Normalize and cap the confidence score between 0 and 1
                confidence = min(1.0, max(0.0, avg_similarity))
            else:
                confidence = 0.0  # Set confidence to 0 when no chunks are retrieved

            # Extract sources that were actually cited in the response text
            # First, look for the "Sources cited:" pattern at the end of the response
            actual_sources = []
            retrieved_source_docs = {chunk.source_document for chunk in retrieved_chunks if chunk.source_document is not None}

            # Check for "Sources cited:" format
            if "Sources cited:" in response_text:
                try:
                    sources_section = response_text.split("Sources cited:")[-1].strip()
                    # Clean up the sources section - remove brackets and extra whitespace
                    sources_section = sources_section.strip('[]')
                    # Split by comma and clean up each source
                    potential_sources = [s.strip().strip('[]') for s in sources_section.split(',')]
                    # Filter out empty strings and get actual sources that exist in retrieved chunks
                    actual_sources = [s for s in potential_sources if s and s in retrieved_source_docs]

                    # If no valid sources found in the extracted list, fall back to all retrieved sources
                    if not actual_sources:
                        actual_sources = list(retrieved_source_docs)
                except Exception:
                    # If parsing fails, fall back to all retrieved sources
                    actual_sources = list(retrieved_source_docs)
            else:
                # If no "Sources cited:" section found, try to extract sources from inline citations
                # Look for patterns like (Source: ..., Page: ...)
                try:
                    # Find all inline citations in the format (Source: [doc], Page: [num])
                    inline_citations = re.findall(r'\(Source:\s*([^,)]+)', response_text)
                    # Filter to only include sources that were actually in the retrieved chunks
                    actual_sources = [citation.strip() for citation in inline_citations if citation.strip() in retrieved_source_docs]

                    # If we found inline citations, use those; otherwise, use all retrieved sources as fallback
                    if not actual_sources:
                        actual_sources = list(retrieved_source_docs)
                except Exception:
                    # If all parsing fails, fall back to all retrieved sources
                    actual_sources = list(retrieved_source_docs)

            # Ensure sources are never empty - fallback to all retrieved source documents if needed
            if not actual_sources:
                actual_sources = list(retrieved_source_docs) if retrieved_source_docs else ["No sources available"]

            # Create response object
            response = QueryResponse(
                id=str(uuid.uuid4()),
                query=query_text,
                response=response_text,
                retrieved_chunks=retrieved_chunks,
                confidence=confidence,
                sources=actual_sources,
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