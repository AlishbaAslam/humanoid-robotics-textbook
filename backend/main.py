import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import time
import hashlib
import logging
from typing import List, Dict, Any, Optional
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import argparse
import re
import uuid
import random
import json
from datetime import datetime, timedelta

# Constants for retrieval testing
DEFAULT_COLLECTION_NAME = "book_embeddings"
DEFAULT_TOP_K = 5
DEFAULT_SIMILARITY_THRESHOLD = 0.0
DEFAULT_NUM_RUNS = 5
DEFAULT_QUERY_TIMEOUT = 10  # seconds

# Constants for rate limiting and embedding generation
EMBEDDING_BATCH_SIZE = 96  # Maximum number of texts per Cohere API request
RATE_LIMIT_DELAY = 1.0  # Base delay in seconds between requests to respect rate limits
MAX_RETRY_ATTEMPTS = 5  # Maximum number of retry attempts for rate limit errors
INITIAL_BACKOFF = 1.0  # Initial backoff time in seconds
MAX_BACKOFF = 60.0  # Maximum backoff time in seconds

# Constants for token budgeting and resumable pipeline
TOKEN_BUDGET_PER_MINUTE = 100000  # Maximum tokens allowed per minute (adjust based on your API plan)
TOKEN_BUFFER = 5000  # Buffer to stay under the limit (5000 tokens less than the maximum)
MIN_TOKENS_PER_REQUEST = 1000  # Minimum tokens per request to ensure efficiency
MAX_COOLDOWN_WINDOW = 300  # Maximum cooldown time in seconds (5 minutes)
PROGRESS_FILE = "embedding_progress.json"  # File to store progress for resumable pipeline

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def get_env_variable(var_name: str, default_value: str = None) -> str:
    """
    Helper function to get environment variables with optional default.

    Args:
        var_name: Name of the environment variable
        default_value: Default value if variable is not set

    Returns:
        Value of the environment variable or default value
    """
    value = os.getenv(var_name, default_value)
    if value is None:
        raise ValueError(f"Required environment variable {var_name} not set")
    return value

def validate_environment() -> bool:
    """
    Validate that all required environment variables are set for retrieval operations.

    Returns:
        True if all required variables are set, False otherwise
    """
    required_vars = ["COHERE_API_KEY", "QDRANT_URL"]
    missing_vars = []

    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        logger.error(f"Missing required environment variables: {', '.join(missing_vars)}")
        return False

    return True

def get_qdrant_client() -> QdrantClient:
    """
    Create and return a Qdrant client instance.

    Returns:
        QdrantClient instance
    """
    # Get Qdrant configuration from environment
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable not set")

    # Initialize Qdrant client
    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        client = QdrantClient(url=qdrant_url)

    return client

def format_retrieval_results(
    search_results: List[Dict[str, Any]],
    query_text: str,
    execution_time: float
) -> Dict[str, Any]:
    """
    Format retrieval results with metadata for testing and logging.

    Args:
        search_results: List of search results from Qdrant
        query_text: Original query text
        execution_time: Time taken to execute the search

    Returns:
        Dictionary containing formatted results with metadata
    """
    formatted_result = {
        "query_text": query_text,
        "execution_time": execution_time,
        "total_results": len(search_results),
        "retrieved_chunks": search_results,
        "retrieval_accuracy": calculate_relevance_score(search_results),
        "metadata_accuracy": calculate_metadata_accuracy(search_results)
    }

    return formatted_result

def calculate_relevance_score(results: List[Dict[str, Any]]) -> float:
    """
    Calculate a basic relevance score based on similarity scores of results.

    Args:
        results: List of search results

    Returns:
        Relevance score between 0 and 1
    """
    if not results:
        return 0.0

    # Calculate average similarity score
    total_score = sum(result.get("similarity_score", 0.0) for result in results)
    avg_score = total_score / len(results)

    # Normalize to 0-1 range (assuming similarity scores are 0-1)
    return min(1.0, max(0.0, avg_score))

def calculate_metadata_accuracy(results: List[Dict[str, Any]]) -> float:
    """
    Calculate metadata accuracy based on completeness of metadata fields.

    Args:
        results: List of search results

    Returns:
        Metadata accuracy score between 0 and 1
    """
    if not results:
        return 0.0

    total_checks = len(results) * 4  # source_url, title, content_hash, chunk_id
    valid_fields = 0

    for result in results:
        if result.get("source_url"):
            valid_fields += 1
        if result.get("title"):
            valid_fields += 1
        if result.get("content_hash"):
            valid_fields += 1
        if result.get("chunk_id"):
            valid_fields += 1

    return valid_fields / total_checks if total_checks > 0 else 0.0

def handle_qdrant_error(error: Exception) -> Dict[str, str]:
    """
    Handle Qdrant-specific errors and return appropriate error information.

    Args:
        error: The exception that occurred

    Returns:
        Dictionary with error details
    """
    error_msg = str(error)
    if "connection" in error_msg.lower() or "timeout" in error_msg.lower():
        return {
            "error": "QDRANT_CONNECTION_ERROR",
            "message": "Unable to connect to Qdrant database",
            "recovery": "Verify Qdrant URL and API key in environment variables"
        }
    elif "collection" in error_msg.lower():
        return {
            "error": "QDRANT_COLLECTION_ERROR",
            "message": "Collection not found or inaccessible",
            "recovery": f"Verify collection exists and name is correct (expected: {DEFAULT_COLLECTION_NAME})"
        }
    else:
        return {
            "error": "QDRANT_GENERAL_ERROR",
            "message": f"Qdrant error occurred: {error_msg}",
            "recovery": "Check Qdrant service status and credentials"
        }

def calculate_exponential_backoff(attempt: int, initial_backoff: float = INITIAL_BACKOFF, max_backoff: float = MAX_BACKOFF) -> float:
    """
    Calculate exponential backoff time with jitter to prevent thundering herd problems.

    Args:
        attempt: Current attempt number (0-indexed)
        initial_backoff: Initial backoff time in seconds
        max_backoff: Maximum backoff time in seconds

    Returns:
        Backoff time in seconds
    """
    # Calculate exponential backoff: initial_backoff * (2^attempt)
    exponential_backoff = initial_backoff * (2 ** attempt)

    # Apply jitter by using a random value between 0.5 and 1.0 of the calculated backoff
    jitter = 0.5 + random.random() * 0.5
    backoff_with_jitter = exponential_backoff * jitter

    # Ensure the backoff doesn't exceed the maximum
    return min(backoff_with_jitter, max_backoff)


def estimate_token_count(text: str) -> int:
    """
    Estimate the number of tokens in a text string.
    This is a rough estimation based on the common rule of thumb that 1 token ≈ 4 characters.

    Args:
        text: Input text to estimate tokens for

    Returns:
        Estimated number of tokens
    """
    if not text:
        return 0
    # Rough estimation: 1 token ≈ 4 characters for English text
    # This is a common approximation; for more accuracy, use Cohere's tokenizer if available
    return len(text) // 4


def calculate_tokens_for_batch(texts: List[str]) -> int:
    """
    Calculate the total estimated token count for a batch of texts.

    Args:
        texts: List of text strings to calculate tokens for

    Returns:
        Total estimated token count
    """
    total_tokens = 0
    for text in texts:
        total_tokens += estimate_token_count(text)
    return total_tokens


class TokenBudgetManager:
    """
    Manages token usage within specified budget limits with time-based tracking.
    """
    def __init__(self, budget_per_minute: int = TOKEN_BUDGET_PER_MINUTE, buffer: int = TOKEN_BUFFER):
        self.budget_per_minute = budget_per_minute
        self.buffer = buffer
        self.effective_budget = budget_per_minute - buffer
        self.used_tokens = 0
        self.window_start_time = datetime.now()

    def can_make_request(self, token_count: int) -> bool:
        """
        Check if we can make a request with the specified token count.

        Args:
            token_count: Number of tokens for the proposed request

        Returns:
            True if the request can be made within budget, False otherwise
        """
        # Check if we're within the current time window (1 minute)
        current_time = datetime.now()
        time_diff = (current_time - self.window_start_time).total_seconds()

        # If more than 1 minute has passed, reset the window
        if time_diff >= 60:
            self.reset_window()

        # Check if adding this request would exceed the budget
        return (self.used_tokens + token_count) <= self.effective_budget

    def record_usage(self, token_count: int):
        """
        Record token usage for a completed request.

        Args:
            token_count: Number of tokens used in the request
        """
        current_time = datetime.now()
        time_diff = (current_time - self.window_start_time).total_seconds()

        # If more than 1 minute has passed, reset the window
        if time_diff >= 60:
            self.reset_window()

        self.used_tokens += token_count

    def get_remaining_budget(self) -> int:
        """
        Get the remaining token budget for the current window.

        Returns:
            Remaining tokens allowed in the current window
        """
        return max(0, self.effective_budget - self.used_tokens)

    def get_wait_time(self) -> float:
        """
        Get the time to wait before the next window starts (if current window is full).

        Returns:
            Time in seconds to wait before next window
        """
        current_time = datetime.now()
        time_elapsed = (current_time - self.window_start_time).total_seconds()
        return max(0, 60 - time_elapsed)

    def reset_window(self):
        """
        Reset the token usage window to start a new minute.
        """
        self.used_tokens = 0
        self.window_start_time = datetime.now()


def calculate_adaptive_batch_size(
    chunks: List[Dict[str, Any]],
    start_idx: int,
    token_budget_manager: TokenBudgetManager,
    max_batch_size: int = EMBEDDING_BATCH_SIZE
) -> int:
    """
    Calculate the adaptive batch size based on token budget constraints.

    Args:
        chunks: List of all text chunks to process
        start_idx: Starting index in the chunks list
        token_budget_manager: Token budget manager to check available tokens
        max_batch_size: Maximum allowed batch size (default from constants)

    Returns:
        Optimal batch size that fits within token budget
    """
    # Check how many tokens we have remaining in our budget
    remaining_tokens = token_budget_manager.get_remaining_budget()

    # If remaining tokens are less than minimum, return 0 (need to wait for next window)
    if remaining_tokens < MIN_TOKENS_PER_REQUEST:
        return 0

    # Start with the maximum possible batch size
    batch_size = min(max_batch_size, len(chunks) - start_idx)

    # Calculate tokens for the full batch
    batch_texts = [chunks[i]["text"] for i in range(start_idx, min(start_idx + batch_size, len(chunks)))]
    estimated_tokens = calculate_tokens_for_batch(batch_texts)

    # If the full batch exceeds the budget, reduce the batch size
    while estimated_tokens > remaining_tokens and batch_size > 1:
        batch_size -= 1
        batch_texts = [chunks[i]["text"] for i in range(start_idx, min(start_idx + batch_size, len(chunks)))]
        estimated_tokens = calculate_tokens_for_batch(batch_texts)

    # Ensure batch size is not zero if there are chunks to process
    if batch_size == 0 and start_idx < len(chunks):
        # If we have remaining tokens but they're not enough for even one chunk,
        # wait for the next token window by returning 0
        return 0

    return batch_size


def save_progress(current_index: int, total_chunks: int, completed_embeddings: List[Dict[str, Any]] = None):
    """
    Save the current progress to a file to allow resuming the embedding process.

    Args:
        current_index: Current chunk index being processed
        total_chunks: Total number of chunks to process
        completed_embeddings: List of embeddings that have been successfully completed
    """
    progress_data = {
        "current_index": current_index,
        "total_chunks": total_chunks,
        "timestamp": datetime.now().isoformat(),
        "completed_count": current_index,
        "remaining_count": total_chunks - current_index
    }

    # Save progress to file
    try:
        with open(PROGRESS_FILE, 'w', encoding='utf-8') as f:
            json.dump(progress_data, f, indent=2)
        logger.info(f"Progress saved: {current_index}/{total_chunks} chunks completed")
    except Exception as e:
        logger.error(f"Failed to save progress: {e}")


def load_progress() -> Dict[str, Any]:
    """
    Load the saved progress from file.

    Returns:
        Dictionary containing progress information or None if no progress file exists
    """
    try:
        if os.path.exists(PROGRESS_FILE):
            with open(PROGRESS_FILE, 'r', encoding='utf-8') as f:
                progress_data = json.load(f)
            logger.info(f"Progress loaded: {progress_data}")
            return progress_data
        else:
            logger.info("No existing progress file found, starting from beginning")
            return None
    except Exception as e:
        logger.error(f"Failed to load progress: {e}")
        return None


def remove_progress_file():
    """
    Remove the progress file when embedding is completed successfully.
    """
    try:
        if os.path.exists(PROGRESS_FILE):
            os.remove(PROGRESS_FILE)
            logger.info("Progress file removed - embedding completed successfully")
    except Exception as e:
        logger.error(f"Failed to remove progress file: {e}")


def make_rate_limited_embedding_request(
    cohere_client,
    texts: List[str],
    model: str,
    input_type: str,
    token_budget_manager: Optional[TokenBudgetManager] = None
) -> Any:
    """
    Make a rate-limited embedding request to Cohere API with retry logic and token budgeting.

    Args:
        cohere_client: Initialized Cohere client
        texts: List of texts to embed
        model: Cohere model to use for embeddings
        input_type: Type of input ("search_document" or "search_query")
        token_budget_manager: Optional token budget manager to enforce token limits

    Returns:
        Cohere API response

    Raises:
        Exception: If all retry attempts fail
    """
    # Calculate token count for this request if using token budgeting
    if token_budget_manager:
        token_count = calculate_tokens_for_batch(texts)

        # Wait if we can't make the request within the token budget
        while not token_budget_manager.can_make_request(token_count):
            wait_time = token_budget_manager.get_wait_time()
            logger.warning(f"Token budget limit reached. Waiting {wait_time:.2f}s for next window...")
            time.sleep(wait_time)
            # Check again after waiting
            if token_budget_manager.can_make_request(token_count):
                break

    last_exception = None

    for attempt in range(MAX_RETRY_ATTEMPTS):
        try:
            # Make the embedding request
            response = cohere_client.embed(
                texts=texts,
                model=model,
                input_type=input_type
            )

            # Record token usage if using token budgeting
            if token_budget_manager:
                token_count = calculate_tokens_for_batch(texts)
                token_budget_manager.record_usage(token_count)

            # If successful, return the response
            return response
        except Exception as e:
            error_msg = str(e).lower()
            # Check if this is a rate limit error (HTTP 429 or similar rate limit indication)
            if "rate limit" in error_msg or "429" in error_msg or "too many requests" in error_msg:
                if attempt < MAX_RETRY_ATTEMPTS - 1:  # Don't sleep on the last attempt
                    backoff_time = calculate_exponential_backoff(attempt)
                    # Enforce maximum cooldown window
                    backoff_time = min(backoff_time, MAX_COOLDOWN_WINDOW)
                    logger.warning(f"Rate limit hit on attempt {attempt + 1}, sleeping for {backoff_time:.2f}s before retry...")
                    time.sleep(backoff_time)
                    continue
                else:
                    logger.error(f"All {MAX_RETRY_ATTEMPTS} retry attempts failed due to rate limiting")
                    logger.info("Enforced cooldown window activated - pipeline will stop and can be resumed")
                    raise e
            else:
                # If it's not a rate limit error, don't retry
                raise e

    # This should never be reached due to the return statement above, but included for completeness
    if last_exception:
        raise last_exception


def enforce_extended_cooldown():
    """
    Enforce an extended cooldown period when repeated rate limit errors occur.
    This function is called to ensure the system respects API limits.
    """
    cooldown_time = MAX_COOLDOWN_WINDOW
    logger.info(f"Enforcing extended cooldown window: {cooldown_time}s")
    time.sleep(cooldown_time)
    logger.info("Extended cooldown window completed")


def handle_cohere_error(error: Exception) -> Dict[str, str]:
    """
    Handle Cohere-specific errors and return appropriate error information.

    Args:
        error: The exception that occurred

    Returns:
        Dictionary with error details
    """
    error_msg = str(error)
    if "api key" in error_msg.lower() or "authentication" in error_msg.lower():
        return {
            "error": "COHERE_API_ERROR",
            "message": "Error with Cohere API for embedding generation",
            "recovery": "Verify Cohere API key in environment variables"
        }
    elif "rate limit" in error_msg.lower():
        return {
            "error": "COHERE_RATE_LIMIT_ERROR",
            "message": "Cohere API rate limit exceeded",
            "recovery": "Wait before making more requests or upgrade your Cohere plan"
        }
    else:
        return {
            "error": "COHERE_GENERAL_ERROR",
            "message": f"Cohere API error occurred: {error_msg}",
            "recovery": "Check Cohere API status and credentials"
        }

def handle_retrieval_error(error: Exception) -> Dict[str, str]:
    """
    Handle general retrieval errors and return appropriate error information.

    Args:
        error: The exception that occurred

    Returns:
        Dictionary with error details
    """
    error_msg = str(error)
    if "no results" in error_msg.lower() or "empty" in error_msg.lower():
        return {
            "error": "NO_RESULTS_FOUND",
            "message": "No results found for the given query",
            "recovery": "Try a different query or verify content exists in the vector store"
        }
    else:
        # Determine if it's more likely a Qdrant or Cohere error based on the error message
        if any(keyword in error_msg.lower() for keyword in ["qdrant", "connection", "timeout", "collection"]):
            return handle_qdrant_error(error)
        elif any(keyword in error_msg.lower() for keyword in ["cohere", "api", "embed", "authentication"]):
            return handle_cohere_error(error)
        else:
            return {
                "error": "RETRIEVAL_GENERAL_ERROR",
                "message": f"Retrieval error occurred: {error_msg}",
                "recovery": "Check logs for more details and verify system configuration"
            }

def retrieve_relevant_chunks(
    query_text: str,
    collection_name: str = DEFAULT_COLLECTION_NAME,
    top_k: int = DEFAULT_TOP_K,
    similarity_threshold: float = DEFAULT_SIMILARITY_THRESHOLD
) -> Dict[str, Any]:
    """
    Main retrieval function that accepts query text and returns relevant chunks from Qdrant.

    Args:
        query_text: Natural language query to search for
        collection_name: Name of the Qdrant collection to search
        top_k: Number of top results to return
        similarity_threshold: Minimum similarity score threshold

    Returns:
        Dictionary containing formatted retrieval results with metadata
    """
    start_time = time.time()

    try:
        # Validate inputs
        if not query_text or not query_text.strip():
            raise ValueError("Query text cannot be empty")

        # Verify collection exists
        if not verify_qdrant_collection(collection_name):
            raise ValueError(f"Collection '{collection_name}' does not exist in Qdrant")

        # Generate embedding for the query
        query_embedding = generate_query_embedding(query_text)

        # Perform similarity search
        search_results = search_qdrant_collection(
            query_embedding,
            collection_name,
            top_k,
            similarity_threshold
        )

        # Calculate execution time
        execution_time = time.time() - start_time

        # Format the results
        formatted_results = format_retrieval_results(search_results, query_text, execution_time)

        return formatted_results

    except Exception as e:
        execution_time = time.time() - start_time
        logger.error(f"Error in retrieve_relevant_chunks: {e}")

        # Format error result
        error_result = {
            "query_text": query_text,
            "execution_time": execution_time,
            "total_results": 0,
            "retrieved_chunks": [],
            "retrieval_accuracy": 0.0,
            "metadata_accuracy": 0.0,
            "error_info": handle_retrieval_error(e)
        }

        return error_result

def test_retrieval_functionality(args: argparse.Namespace):
    """
    Test the retrieval functionality with the provided arguments.

    Args:
        args: Parsed command-line arguments
    """
    logger.info("Starting retrieval functionality test...")

    # Validate environment
    if not validate_environment():
        logger.error("Environment validation failed. Please check your configuration.")
        return

    # Test with a single query if provided
    if args.query:
        logger.info(f"Testing query: {args.query}")
        results = retrieve_relevant_chunks(
            query_text=args.query,
            collection_name=args.collection,
            top_k=args.top_k,
            similarity_threshold=args.similarity_threshold
        )

        # Log the results
        logger.info(f"Query: {results['query_text']}")
        logger.info(f"Execution time: {results['execution_time']:.4f} seconds")
        logger.info(f"Total results: {results['total_results']}")
        logger.info(f"Retrieval accuracy: {results['retrieval_accuracy']:.2f}")
        logger.info(f"Metadata accuracy: {results['metadata_accuracy']:.2f}")

        if results.get('error_info'):
            logger.error(f"Error: {results['error_info']['message']}")
            logger.info(f"Recovery suggestion: {results['error_info']['recovery']}")
        else:
            # Display top results
            logger.info("Top results:")
            for i, chunk in enumerate(results['retrieved_chunks'][:3]):  # Show top 3
                logger.info(f"  {i+1}. Score: {chunk['similarity_score']:.3f}")
                logger.info(f"     Source: {chunk['source_url']}")
                logger.info(f"     Title: {chunk['title']}")
                logger.info(f"     Preview: {chunk['text'][:100]}...")
                logger.info("")

    elif args.queries_file:
        # Test with queries from a file
        logger.info(f"Testing queries from file: {args.queries_file}")
        try:
            with open(args.queries_file, 'r', encoding='utf-8') as f:
                queries = [line.strip() for line in f if line.strip()]

            for i, query in enumerate(queries):
                logger.info(f"Processing query {i+1}/{len(queries)}: {query}")
                results = retrieve_relevant_chunks(
                    query_text=query,
                    collection_name=args.collection,
                    top_k=args.top_k,
                    similarity_threshold=args.similarity_threshold
                )

                logger.info(f"  Results: {results['total_results']}, "
                           f"Accuracy: {results['retrieval_accuracy']:.2f}, "
                           f"Time: {results['execution_time']:.4f}s")
        except FileNotFoundError:
            logger.error(f"Queries file not found: {args.queries_file}")
        except Exception as e:
            logger.error(f"Error reading queries file: {e}")
    else:
        logger.warning("No query provided. Use --query 'your query' or --queries-file 'path/to/file'")

def test_metadata_integrity(args: argparse.Namespace):
    """
    Test the metadata integrity in retrieved results.

    Args:
        args: Parsed command-line arguments
    """
    logger.info("Starting metadata integrity test...")

    # Validate environment
    if not validate_environment():
        logger.error("Environment validation failed. Please check your configuration.")
        return

    # Test with a single query if provided
    if args.query:
        logger.info(f"Testing metadata integrity for query: {args.query}")
        results = retrieve_relevant_chunks(
            query_text=args.query,
            collection_name=args.collection,
            top_k=args.top_k,
            similarity_threshold=args.similarity_threshold
        )

        logger.info(f"Query: {results['query_text']}")
        logger.info(f"Metadata accuracy: {results['metadata_accuracy']:.2f}")

        if results.get('error_info'):
            logger.error(f"Error: {results['error_info']['message']}")
            logger.info(f"Recovery suggestion: {results['error_info']['recovery']}")
        else:
            # Validate each result's metadata
            for i, chunk in enumerate(results['retrieved_chunks']):
                logger.info(f"Result {i+1}:")
                logger.info(f"  Source URL: {'✓' if chunk['source_url'] else '✗'} {chunk['source_url']}")
                logger.info(f"  Title: {'✓' if chunk['title'] else '✗'} {chunk['title']}")
                logger.info(f"  Content Hash: {'✓' if chunk['content_hash'] else '✗'} {chunk['content_hash'][:10]}...")
                logger.info(f"  Chunk ID: {'✓' if chunk['chunk_id'] else '✗'} {chunk['chunk_id']}")
                logger.info(f"  Similarity Score: {chunk['similarity_score']:.3f}")
                logger.info("")
    else:
        logger.warning("No query provided for metadata testing. Use --query 'your query'")

def test_retrieval_consistency(args: argparse.Namespace):
    """
    Test the consistency of retrieval results across multiple runs.

    Args:
        args: Parsed command-line arguments
    """
    logger.info("Starting retrieval consistency test...")

    # Validate environment
    if not validate_environment():
        logger.error("Environment validation failed. Please check your configuration.")
        return

    if not args.query:
        logger.warning("No query provided for consistency testing. Use --query 'your query'")
        return

    logger.info(f"Testing consistency for query: {args.query} over {args.runs} runs")

    all_results = []
    execution_times = []

    for run in range(args.runs):
        logger.info(f"Run {run + 1}/{args.runs}")
        results = retrieve_relevant_chunks(
            query_text=args.query,
            collection_name=args.collection,
            top_k=args.top_k,
            similarity_threshold=args.similarity_threshold
        )

        all_results.append(results)
        execution_times.append(results['execution_time'])

        if results.get('error_info'):
            logger.error(f"Run {run + 1} failed: {results['error_info']['message']}")
        else:
            logger.info(f"  Results: {results['total_results']}, "
                       f"Accuracy: {results['retrieval_accuracy']:.2f}, "
                       f"Time: {results['execution_time']:.4f}s")

    # Analyze consistency
    if all_results and not any(result.get('error_info') for result in all_results):
        # Calculate average execution time
        avg_time = sum(execution_times) / len(execution_times)
        time_variance = sum((t - avg_time) ** 2 for t in execution_times) / len(execution_times)

        logger.info(f"\nConsistency Analysis:")
        logger.info(f"  Average execution time: {avg_time:.4f}s")
        logger.info(f"  Time variance: {time_variance:.6f}")
        logger.info(f"  Min time: {min(execution_times):.4f}s")
        logger.info(f"  Max time: {max(execution_times):.4f}s")

        # Check result consistency (comparing first result across runs)
        first_chunks = [result['retrieved_chunks'][0]['content_hash']
                       if result['retrieved_chunks'] else None
                       for result in all_results]

        unique_chunks = set(first_chunks)
        consistency_percentage = (len(first_chunks) - len(unique_chunks) + 1) / len(first_chunks) * 100

        logger.info(f"  Consistency of top result: {consistency_percentage:.2f}%")

        # Calculate average retrieval and metadata accuracy
        avg_retrieval_acc = sum(r['retrieval_accuracy'] for r in all_results) / len(all_results)
        avg_metadata_acc = sum(r['metadata_accuracy'] for r in all_results) / len(all_results)

        logger.info(f"  Average retrieval accuracy: {avg_retrieval_acc:.2f}")
        logger.info(f"  Average metadata accuracy: {avg_metadata_acc:.2f}")
    else:
        logger.warning("Could not perform consistency analysis due to errors in runs")

def run_comprehensive_test_suite(args: argparse.Namespace):
    """
    Run the comprehensive test suite for retrieval functionality.

    Args:
        args: Parsed command-line arguments
    """
    logger.info("Starting comprehensive retrieval test suite...")

    # Validate environment
    if not validate_environment():
        logger.error("Environment validation failed. Please check your configuration.")
        return

    if not args.query:
        # Use a default query for the test suite if none provided
        args.query = "What is ROS2 middleware architecture?"
        logger.info(f"No query provided, using default: {args.query}")

    logger.info("Running all tests in sequence...")

    # Run basic retrieval test
    logger.info("\n1. Running basic retrieval test...")
    test_retrieval_functionality(args)

    # Run metadata integrity test
    logger.info("\n2. Running metadata integrity test...")
    test_metadata_integrity(args)

    # Run consistency test
    logger.info("\n3. Running consistency test...")
    test_retrieval_consistency(args)

    logger.info("\nComprehensive test suite completed!")

def search_qdrant_collection(
    query_embedding: List[float],
    collection_name: str = DEFAULT_COLLECTION_NAME,
    top_k: int = DEFAULT_TOP_K,
    similarity_threshold: float = DEFAULT_SIMILARITY_THRESHOLD
) -> List[Dict[str, Any]]:
    """
    Perform vector similarity search in Qdrant collection.

    Args:
        query_embedding: The embedding vector to search for
        collection_name: Name of the collection to search (default from constants)
        top_k: Number of top results to return (default from constants)
        similarity_threshold: Minimum similarity score threshold (default from constants)

    Returns:
        List of dictionaries containing search results with text, metadata, and similarity scores
    """
    try:
        client = get_qdrant_client()

        # Perform the search in Qdrant using the query_points method for vector search
        search_results = client.query_points(
            collection_name=collection_name,
            query=query_embedding,  # Pass the embedding vector as query
            limit=top_k,
            score_threshold=similarity_threshold  # Filter results by similarity threshold
        )

        # Format the results
        formatted_results = []
        for result in search_results.points:
            formatted_result = {
                "text": result.payload.get("text", ""),
                "source_url": result.payload.get("source_url", ""),
                "title": result.payload.get("title", ""),
                "content_hash": result.payload.get("content_hash", ""),
                "chunk_id": result.payload.get("chunk_id", ""),
                "similarity_score": result.score,
                "id": result.id
            }
            formatted_results.append(formatted_result)

        return formatted_results
    except Exception as e:
        logger.error(f"Error performing Qdrant search: {e}")
        raise

def verify_qdrant_collection(collection_name: str = DEFAULT_COLLECTION_NAME) -> bool:
    """
    Verify that the specified Qdrant collection exists.

    Args:
        collection_name: Name of the collection to verify (default from constants)

    Returns:
        True if collection exists, False otherwise
    """
    try:
        client = get_qdrant_client()

        # Check if collection exists
        collections = client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        return collection_exists
    except Exception as e:
        logger.error(f"Error verifying Qdrant collection: {e}")
        return False

def generate_query_embedding(query_text: str, model: str = "embed-multilingual-v3.0") -> List[float]:
    """
    Generate embeddings for a query text using Cohere API with rate limiting and retry logic.

    Args:
        query_text: The query text to embed
        model: Cohere model to use for embeddings (default "embed-multilingual-v3.0")

    Returns:
        List of embedding values as floats
    """
    if not query_text or not query_text.strip():
        raise ValueError("Query text cannot be empty")

    # Initialize Cohere client
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    co = cohere.Client(cohere_api_key)

    try:
        # Generate embedding for the query with rate limiting and retry logic
        response = make_rate_limited_embedding_request(
            cohere_client=co,
            texts=[query_text],
            model=model,
            input_type="search_query"  # Using search_query as the input type for queries
        )

        return response.embeddings[0]  # Return the first (and only) embedding
    except Exception as e:
        logger.error(f"Error generating query embedding: {e}")
        raise

def fetch_all_urls(base_url: str, max_pages: int = 500) -> List[str]:
    """
    Fetches all discoverable URLs from a Docusaurus site.

    Args:
        base_url (str): The base URL of the Docusaurus site to crawl
        max_pages (int): Maximum number of pages to crawl (default 500)

    Returns:
        List[str]: List of valid URLs discovered on the site
    """
    urls = set()
    to_visit = [base_url]
    visited = set()

    # Parse the base domain to only crawl within the same domain
    base_domain = urlparse(base_url).netloc

    while to_visit and len(urls) < max_pages:
        current_url = to_visit.pop(0)

        if current_url in visited:
            continue

        visited.add(current_url)
        urls.add(current_url)

        try:
            # Add a small delay to be respectful to the server
            time.sleep(0.1)

            response = requests.get(current_url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, 'html.parser')

            # Find all links in the page
            for link in soup.find_all('a', href=True):
                href = link['href']
                absolute_url = urljoin(current_url, href)

                # Only add URLs from the same domain and that are likely to be content pages
                parsed_url = urlparse(absolute_url)
                if (parsed_url.netloc == base_domain and
                    not absolute_url.endswith(('.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.exe')) and
                    not 'mailto:' in absolute_url and
                    not 'tel:' in absolute_url):

                    # Check if it's a likely content page
                    if (absolute_url.startswith(base_url) and
                        absolute_url not in visited and
                        len(urls) < max_pages):
                        to_visit.append(absolute_url)

        except requests.RequestException as e:
            logger.warning(f"Error fetching {current_url}: {e}")
            continue
        except Exception as e:
            logger.warning(f"Error parsing {current_url}: {e}")
            continue

    return list(urls)


def get_text_from_url(url: str) -> Dict[str, Any]:
    """
    Extracts and cleans text content from a given URL.

    Args:
        url (str): The URL to extract content from

    Returns:
        Dict[str, Any]: Dictionary containing extracted content and metadata
    """
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.text, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Try to find the main content area (Docusaurus specific selectors)
        content_selectors = [
            'article',  # Common for content
            'main',     # Main content area
            '.main-wrapper',  # Docusaurus specific
            '.container',     # Common container
            '.theme-doc-markdown',  # Docusaurus markdown content
            '.markdown',      # Markdown content area
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        # If no specific content area found, use the body
        if not content_element:
            content_element = soup.find('body')

        # Extract text from the content element
        if content_element:
            text = content_element.get_text(separator=' ', strip=True)
        else:
            text = soup.get_text(separator=' ', strip=True)

        # Clean up the text (remove extra whitespace, newlines)
        text = re.sub(r'\s+', ' ', text).strip()

        # Extract title
        title_tag = soup.find('title')
        title = title_tag.get_text().strip() if title_tag else "No Title"

        # Calculate content hash
        content_hash = hashlib.sha256(text.encode('utf-8')).hexdigest()

        # Calculate word count
        word_count = len(text.split())

        # Extract additional metadata
        metadata = {}
        description_tag = soup.find('meta', attrs={'name': 'description'})
        if description_tag and description_tag.get('content'):
            metadata['description'] = description_tag.get('content')

        return {
            "url": url,
            "title": title,
            "text": text,
            "word_count": word_count,
            "content_hash": content_hash,
            "metadata": metadata
        }
    except requests.RequestException as e:
        logger.error(f"Error fetching content from {url}: {e}")
        raise
    except Exception as e:
        logger.error(f"Error extracting content from {url}: {e}")
        raise ValueError(f"Content extraction failed for {url}")


def split_text(text: str, max_chunk_size: int = 4000, overlap: int = 200) -> List[Dict[str, Any]]:
    """
    Splits text into smaller chunks while preserving semantic coherence.

    Args:
        text (str): The text to split
        max_chunk_size (int): Maximum size of each chunk in characters (default 4000)
        overlap (int): Number of characters to overlap between chunks (default 200)

    Returns:
        List[Dict[str, Any]]: List of chunk dictionaries
    """
    if not text or len(text.strip()) == 0:
        # Return a single empty chunk instead of raising an error
        # This ensures that every URL contributes at least one vector to Qdrant
        empty_hash = hashlib.sha256("".encode('utf-8')).hexdigest()
        return [{
            "text": "",
            "chunk_index": 0,
            "word_count": 0,
            "content_hash": empty_hash
        }]

    if max_chunk_size <= 0:
        raise ValueError("max_chunk_size must be positive")

    if overlap < 0 or overlap >= max_chunk_size:
        raise ValueError("overlap must be non-negative and less than max_chunk_size")

    chunks = []
    start = 0
    chunk_index = 0

    while start < len(text):
        # Find the end position for this chunk
        end = start + max_chunk_size

        # If we're at the end of the text, use the remaining text
        if end >= len(text):
            end = len(text)
        else:
            # Try to break at sentence boundaries if possible
            # Look for a sentence end (., !, ?) near the end of the chunk
            chunk_text = text[start:end]
            sentence_end_pos = -1

            # Look for sentence endings in the last 200 characters of the chunk
            lookback_range = min(200, len(chunk_text))
            for punct in ['.', '!', '?', '\n']:
                last_punct = chunk_text.rfind(punct, -lookback_range)
                if last_punct > sentence_end_pos:
                    sentence_end_pos = last_punct

            if sentence_end_pos > 0:
                end = start + sentence_end_pos + 1  # Include the punctuation

        # Extract the chunk
        chunk_text = text[start:end]
        chunk_word_count = len(chunk_text.split())
        chunk_hash = hashlib.sha256(chunk_text.encode('utf-8')).hexdigest()

        chunks.append({
            "text": chunk_text,
            "chunk_index": chunk_index,
            "word_count": chunk_word_count,
            "content_hash": chunk_hash
        })

        # Move to the next chunk position (with overlap)
        start = end - overlap if end < len(text) else len(text)
        chunk_index += 1

    return chunks


def generate_embeddings(chunks: List[Dict[str, Any]], model: str = "embed-multilingual-v3.0") -> List[Dict[str, Any]]:
    """
    Generates vector embeddings for text chunks using Cohere API with token budgeting,
    adaptive batch sizing, progress persistence, and rate limiting.

    Args:
        chunks (List[Dict[str, Any]]): List of text chunks to embed
        model (str): Cohere model to use for embeddings (default "embed-multilingual-v3.0")

    Returns:
        List[Dict[str, Any]]: List of embedding dictionaries
    """
    if not chunks:
        raise ValueError("Chunks list cannot be empty")

    # Initialize Cohere client
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    co = cohere.Client(cohere_api_key)

    # Initialize token budget manager
    token_budget_manager = TokenBudgetManager()

    # Check for existing progress to resume from
    progress_data = load_progress()
    start_index = 0
    all_embeddings = []

    if progress_data:
        start_index = progress_data.get("current_index", 0)
        logger.info(f"Resuming from chunk {start_index} of {progress_data.get('total_chunks', len(chunks))}")

        # If resuming, we need to reconstruct the already completed embeddings
        # For this implementation, we'll just log that we're resuming and continue
        logger.info(f"Resuming embedding generation from chunk {start_index}")

    # Prepare the text for embedding
    total_chunks = len(chunks)

    # Generate embeddings using Cohere with token budgeting and adaptive batch sizing
    try:
        i = start_index
        while i < len(chunks):
            # Calculate adaptive batch size based on token budget
            batch_size = calculate_adaptive_batch_size(
                chunks,
                i,
                token_budget_manager,
                max_batch_size=EMBEDDING_BATCH_SIZE
            )

            # If batch size is 0, it means we need to wait for the next token window
            if batch_size == 0:
                wait_time = token_budget_manager.get_wait_time()
                logger.info(f"Waiting for token budget reset: {wait_time:.2f}s remaining in window")
                time.sleep(wait_time)
                # Continue to recalculate batch size after the window resets
                continue

            # Prepare batch texts
            batch_texts = [chunks[j]["text"] for j in range(i, min(i + batch_size, len(chunks)))]

            try:
                # Make rate-limited embedding request with retry logic and token budgeting
                response = make_rate_limited_embedding_request(
                    cohere_client=co,
                    texts=batch_texts,
                    model=model,
                    input_type="search_document",  # Using search_document as the input type for content
                    token_budget_manager=token_budget_manager
                )

                # Process the batch results
                for j, embedding_vector in enumerate(response.embeddings):
                    chunk_idx = i + j
                    if chunk_idx < len(chunks):  # Safety check
                        chunk = chunks[chunk_idx]
                        all_embeddings.append({
                            "chunk_id": f"{chunk['content_hash']}_{chunk['chunk_index']}",
                            "vector": embedding_vector,
                            "text": chunk["text"],
                            "content_hash": chunk["content_hash"]
                        })

                # Update and save progress
                i += len(response.embeddings)
                save_progress(i, total_chunks)

                logger.info(f"Processed batch: {i}/{total_chunks} chunks completed")

                # Add a small delay between batches to respect rate limits
                if i < len(chunks):  # Only sleep if there are more chunks to process
                    time.sleep(RATE_LIMIT_DELAY)

            except Exception as e:
                error_msg = str(e).lower()
                # Check if this is a rate limit error that requires enforced cooldown
                if "rate limit" in error_msg or "429" in error_msg or "too many requests" in error_msg:
                    logger.error(f"Rate limit error at chunk {i}: {e}")
                    logger.info("Pipeline stopping gracefully due to rate limits. Progress saved. Resume with same parameters.")
                    raise e
                else:
                    # For other errors, log and continue with next batch
                    logger.error(f"Error processing batch starting at chunk {i}: {e}")
                    # Skip this batch and continue with next
                    i += batch_size
                    save_progress(i, total_chunks)

        # If we successfully complete all chunks, remove the progress file
        remove_progress_file()
        logger.info(f"Embedding generation completed successfully: {len(all_embeddings)} chunks processed")
        return all_embeddings

    except KeyboardInterrupt:
        logger.info("Embedding generation interrupted by user. Progress saved.")
        raise
    except Exception as e:
        logger.error(f"Error generating embeddings: {e}")
        # Don't remove progress file on error - allows resuming from failure point
        raise


def create_rag_collection(collection_name: str = "book_embeddings", vector_size: int = 1024) -> bool:
    """
    Creates or ensures existence of the Qdrant collection for embeddings.

    Args:
        collection_name (str): Name of the collection (default "book_embeddings")
        vector_size (int): Dimension of the embedding vectors (default 1024)

    Returns:
        bool: True if collection exists or was created successfully
    """
    # Get Qdrant configuration from environment
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable not set")

    # Initialize Qdrant client
    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        client = QdrantClient(url=qdrant_url)

    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if not collection_exists:
            # Create the collection with specified vector size
            client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE)
            )
            logger.info(f"Created Qdrant collection: {collection_name}")
        else:
            logger.info(f"Qdrant collection already exists: {collection_name}")

        return True
    except Exception as e:
        logger.error(f"Error creating Qdrant collection: {e}")
        raise


def store_chunks_in_qdrant(
    embeddings: List[Dict[str, Any]],
    collection_name: str = "book_embeddings"
) -> Dict[str, Any]:
    """
    Stores vector embeddings in Qdrant with associated metadata.

    Args:
        embeddings (List[Dict[str, Any]]): List of embedding dictionaries
        collection_name (str): Name of the Qdrant collection (default "book_embeddings")

    Returns:
        Dict[str, Any]: Storage result summary
    """
    if not embeddings:
        raise ValueError("Embeddings list cannot be empty")

    # Get Qdrant configuration from environment
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable not set")

    # Initialize Qdrant client
    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    else:
        client = QdrantClient(url=qdrant_url)

    try:
        points = []
        for i, embedding_data in enumerate(embeddings):
            # Create a Qdrant point with the embedding and metadata
            # Use a UUID to avoid conflicts with existing points
            point_id = str(uuid.uuid4())  # Using UUID to ensure unique IDs

            point = models.PointStruct(
                id=point_id,
                vector=embedding_data["vector"],
                payload={
                    "text": embedding_data["text"],
                    "content_hash": embedding_data["content_hash"],
                    "original_chunk_index": i,
                    "chunk_id": embedding_data["chunk_id"],  # Store original chunk_id in payload
                    "source_url": embedding_data.get("source_url", ""),  # Include source URL if available
                    "title": embedding_data.get("title", "")  # Include title if available
                }
            )
            points.append(point)

        # Upload the points to Qdrant
        client.upsert(
            collection_name=collection_name,
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
        return {
            "total_processed": len(embeddings),
            "successful": 0,
            "failed": len(embeddings),
            "errors": [str(e)]
        }


def main():
    """
    Main function to orchestrate the complete workflow:
    1. Fetch all URLs from the Docusaurus site
    2. Extract content from each URL
    3. Split content into chunks
    4. Generate embeddings for chunks
    5. Store embeddings in Qdrant
    """
    parser = argparse.ArgumentParser(description='Docusaurus Content Ingestion and Retrieval Pipeline')
    parser.add_argument('--url', type=str, help='Base URL of the Docusaurus site to crawl')
    parser.add_argument('--collection', type=str, default=DEFAULT_COLLECTION_NAME, help='Qdrant collection name')
    parser.add_argument('--chunk-size', type=int, default=4000, help='Maximum character size for text chunks')
    parser.add_argument('--overlap', type=int, default=200, help='Character overlap between chunks')
    parser.add_argument('--max-pages', type=int, default=500, help='Maximum number of pages to crawl')

    # Retrieval testing arguments
    parser.add_argument('--test-retrieval', action='store_true', help='Run retrieval testing with specified query')
    parser.add_argument('--query', type=str, help='Natural language query to search for in the vector store')
    parser.add_argument('--test-metadata', action='store_true', help='Validate metadata integrity in retrieved results')
    parser.add_argument('--test-consistency', action='store_true', help='Test consistency of retrieval results across multiple runs')
    parser.add_argument('--test-all', action='store_true', help='Run comprehensive test suite')
    parser.add_argument('--queries-file', type=str, help='File containing multiple queries to test')
    parser.add_argument('--runs', type=int, default=DEFAULT_NUM_RUNS, help='Number of runs for consistency testing')
    parser.add_argument('--top-k', type=int, default=DEFAULT_TOP_K, help='Number of top results to return')
    parser.add_argument('--similarity-threshold', type=float, default=DEFAULT_SIMILARITY_THRESHOLD, help='Minimum similarity score threshold')

    args = parser.parse_args()

    # Check if this is a retrieval testing request
    if (args.test_retrieval or args.test_metadata or
        args.test_consistency or args.test_all):

        # Handle retrieval testing
        try:
            # Connect to Qdrant and run tests
            if args.test_all:
                logger.info("Running comprehensive retrieval test suite...")
                run_comprehensive_test_suite(args)
            elif args.test_consistency:
                logger.info(f"Testing consistency with query: {args.query or 'None'}")
                test_retrieval_consistency(args)
            elif args.test_metadata:
                logger.info(f"Testing metadata integrity with query: {args.query or 'None'}")
                test_metadata_integrity(args)
            elif args.test_retrieval:
                logger.info(f"Testing retrieval with query: {args.query or 'None'}")
                test_retrieval_functionality(args)
        except Exception as e:
            logger.error(f"Retrieval testing failed with error: {e}")
            raise
    else:
        # Use the provided URL or fall back to environment variable for ingestion
        base_url = args.url or os.getenv("BASE_URL")
        if not base_url:
            raise ValueError("No base URL provided. Use --url argument or set BASE_URL environment variable.")

        logger.info(f"Starting ingestion pipeline for: {base_url}")

        try:
            # Step 1: Fetch all URLs
            logger.info("Step 1: Fetching all URLs from the site...")
            urls = fetch_all_urls(base_url, args.max_pages)
            logger.info(f"Found {len(urls)} URLs to process")

            # Step 2: Extract content from each URL
            logger.info("Step 2: Extracting content from URLs...")
            all_chunks = []
            empty_content_count = 0
            for i, url in enumerate(urls):
                logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")
                try:
                    content_data = get_text_from_url(url)
                    chunks = split_text(content_data["text"], args.chunk_size, args.overlap)

                    # Add source metadata to each chunk
                    for chunk in chunks:
                        chunk["source_url"] = url
                        chunk["title"] = content_data["title"]
                        chunk["content_hash"] = content_data["content_hash"]

                        # Track if this chunk is empty
                        if not chunk["text"].strip():
                            empty_content_count += 1
                            logger.debug(f"URL {url} produced empty content chunk")

                    all_chunks.extend(chunks)
                except Exception as e:
                    logger.error(f"Error processing URL {url}: {e}")
                    # Create a fallback empty chunk to ensure every URL is represented
                    empty_hash = hashlib.sha256(f"{url}".encode('utf-8')).hexdigest()
                    fallback_chunk = {
                        "text": "",
                        "chunk_index": 0,
                        "word_count": 0,
                        "content_hash": empty_hash,
                        "source_url": url,
                        "title": f"Error processing {url}"
                    }
                    all_chunks.append(fallback_chunk)
                    empty_content_count += 1
                    continue

            logger.info(f"Extracted {len(all_chunks)} content chunks from {len(urls)} URLs")
            logger.info(f"Number of chunks with empty content: {empty_content_count}")

            if not all_chunks:
                logger.warning("No content chunks extracted. Exiting.")
                return

            # Step 3: Generate embeddings
            logger.info("Step 3: Generating embeddings...")
            embeddings = generate_embeddings(all_chunks)
            logger.info(f"Generated embeddings for {len(embeddings)} chunks")

            # Step 4: Create Qdrant collection
            logger.info("Step 4: Creating Qdrant collection...")
            create_rag_collection(args.collection)

            # Step 5: Store embeddings in Qdrant
            logger.info("Step 5: Storing embeddings in Qdrant...")
            result = store_chunks_in_qdrant(embeddings, args.collection)

            logger.info(f"Storage completed. Successful: {result['successful']}, Failed: {result['failed']}")

            if result['errors']:
                logger.error(f"Storage errors: {result['errors']}")

            logger.info("Ingestion pipeline completed successfully!")

        except Exception as e:
            logger.error(f"Pipeline failed with error: {e}")
            raise


if __name__ == "__main__":
    main()