from fastapi import APIRouter, HTTPException
from typing import Dict, Any
from src.models.query import QueryRequest, QueryResponse, BatchQueryRequest, BatchQueryResponse
from src.agents.retrieval_agent import retrieval_agent
from src.config.constants import ERROR_AGENT_PROCESSING
import logging
import uuid
from datetime import datetime

logger = logging.getLogger(__name__)

# Greeting detection
GREETINGS = [
    "hello", "hi", "hey", "greetings", "good morning", "good afternoon",
    "good evening", "good day", "howdy", "hola", "bonjour", "ciao",
    "hallo", "namaste", "konichiwa", "guten tag", "hej", "salut"
]

import re

def handle_greetings(query: str) -> bool:
    """
    Check if the query is a greeting.

    Args:
        query: The user's query string

    Returns:
        True if the query is a greeting, False otherwise
    """
    query_lower = query.lower().strip()
    for greeting in GREETINGS:
        # Use word boundaries to match whole words only
        if re.search(r'\b' + re.escape(greeting) + r'\b', query_lower):
            return True
    return False

router = APIRouter()

@router.post("/query", summary="Submit a query to the agent")
async def submit_query(query_request: QueryRequest) -> QueryResponse:
    """
    Process a user query and return a response based on humanoid robotics content
    """
    try:
        # Check if the query is a greeting before processing with retrieval agent
        if handle_greetings(query_request.query):
            # Return a friendly greeting response
            greeting_response = QueryResponse(
                id=str(uuid.uuid4()),
                query=query_request.query,
                response="Hello! I'm your humanoid robotics assistant. How can I help you with information about humanoid robots today?",
                retrieved_chunks=[],
                confidence=None,
                sources=["greeting"],
                timestamp=datetime.now()
            )
            return greeting_response

        # Process the query using the retrieval agent
        response = await retrieval_agent.process_query(query_request)
        return response
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing query: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail={
                "error": "QUERY_PROCESSING_ERROR",
                "message": "Failed to process query due to internal error"
            }
        )


@router.post("/query/batch", summary="Submit multiple queries to the agent")
async def submit_batch_query(batch_request: BatchQueryRequest) -> BatchQueryResponse:
    """
    Process multiple user queries in sequence
    """
    try:
        responses = []
        for query_req in batch_request.queries:
            # Process each query individually
            query_req.session_id = batch_request.session_id or query_req.session_id

            # Check if the query is a greeting before processing with retrieval agent
            if handle_greetings(query_req.query):
                # Return a friendly greeting response
                greeting_response = QueryResponse(
                    id=str(uuid.uuid4()),
                    query=query_req.query,
                    response="Hello! I'm your humanoid robotics assistant. How can I help you with information about humanoid robots today?",
                    retrieved_chunks=[],
                    confidence=None,
                    sources=["greeting"],
                    timestamp=datetime.now()
                )
                responses.append(greeting_response)
            else:
                response = await retrieval_agent.process_query(query_req)
                responses.append(response)

        return BatchQueryResponse(
            responses=responses,
            session_id=batch_request.session_id
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error processing batch query: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail={
                "error": "BATCH_QUERY_PROCESSING_ERROR",
                "message": "Failed to process batch queries due to internal error"
            }
        )


@router.get("/session/{session_id}", summary="Get session state")
async def get_session_state(session_id: str) -> Dict[str, Any]:
    """
    Get the current state of a specific session
    """
    try:
        state = await retrieval_agent.get_session_state(session_id)
        if state is None:
            raise HTTPException(
                status_code=404,
                detail="Session not found"
            )
        return state.dict()
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting session state: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Failed to retrieve session state"
        )


@router.delete("/session/{session_id}", summary="Clear session")
async def clear_session(session_id: str) -> Dict[str, str]:
    """
    Clear the conversation history for a specific session
    """
    try:
        await retrieval_agent.clear_session(session_id)
        return {"message": f"Session {session_id} cleared successfully"}
    except Exception as e:
        logger.error(f"Error clearing session: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="Failed to clear session"
        )