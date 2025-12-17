import hashlib
import uuid
from typing import Any, Dict, List
from datetime import datetime
import time
import logging


def generate_content_hash(content: str) -> str:
    """
    Generate a SHA-256 hash for content deduplication and identification.
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def generate_unique_id(prefix: str = "") -> str:
    """
    Generate a unique identifier, optionally with a prefix.
    """
    unique_id = str(uuid.uuid4())
    return f"{prefix}{unique_id}" if prefix else unique_id


def calculate_similarity_score(text1: str, text2: str) -> float:
    """
    Calculate a basic similarity score between two texts (0.0 to 1.0).
    This is a simple implementation; for production, consider using more sophisticated algorithms.
    """
    if not text1 or not text2:
        return 0.0

    # Convert to sets of words for basic similarity calculation
    words1 = set(text1.lower().split())
    words2 = set(text2.lower().split())

    if not words1 and not words2:
        return 1.0
    if not words1 or not words2:
        return 0.0

    # Calculate Jaccard similarity
    intersection = len(words1.intersection(words2))
    union = len(words1.union(words2))

    return intersection / union if union > 0 else 0.0


def format_timestamp(dt: datetime) -> str:
    """
    Format a datetime object to ISO string format.
    """
    return dt.isoformat()


def log_execution_time(func_name: str = ""):
    """
    Decorator to log execution time of functions.
    """
    def decorator(func):
        async def wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = await func(*args, **kwargs)
                end_time = time.time()
                execution_time = end_time - start_time
                logging.info(f"{func_name or func.__name__} executed in {execution_time:.4f}s")
                return result
            except Exception as e:
                end_time = time.time()
                execution_time = end_time - start_time
                logging.error(f"{func_name or func.__name__} failed after {execution_time:.4f}s: {e}")
                raise
        return wrapper
    return decorator


def extract_sources_from_chunks(chunks: List[Any]) -> List[str]:
    """
    Extract unique source documents from a list of retrieved chunks.
    """
    sources = set()
    for chunk in chunks:
        if hasattr(chunk, 'source_document') and chunk.source_document:
            sources.add(chunk.source_document)
    return list(sources)


def format_retrieved_chunks_for_prompt(chunks: List[Any]) -> str:
    """
    Format retrieved chunks into a string suitable for inclusion in a prompt.
    """
    if not chunks:
        return "No relevant information found."

    formatted_chunks = []
    for i, chunk in enumerate(chunks, 1):
        chunk_text = f"Source: {getattr(chunk, 'source_document', 'Unknown')}\n"
        chunk_text += f"Content: {getattr(chunk, 'content', '')}\n"
        if hasattr(chunk, 'page_number') and chunk.page_number:
            chunk_text += f"Page: {chunk.page_number}\n"
        chunk_text += "---\n"
        formatted_chunks.append(chunk_text)

    return "".join(formatted_chunks)


def calculate_confidence_score(chunks: List[Any]) -> float:
    """
    Calculate an overall confidence score based on retrieved chunks.
    """
    if not chunks:
        return 0.0

    # Calculate average similarity score
    total_score = sum(getattr(chunk, 'similarity_score', 0.0) for chunk in chunks)
    avg_score = total_score / len(chunks)

    # Normalize to 0-1 range
    return min(1.0, max(0.0, avg_score * 1.5))  # Boost slightly to account for relevance


def paginate_results(results: List[Any], page: int, page_size: int) -> List[Any]:
    """
    Paginate a list of results.
    """
    start_idx = (page - 1) * page_size
    end_idx = start_idx + page_size
    return results[start_idx:end_idx]