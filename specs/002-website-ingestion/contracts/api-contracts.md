# API Contracts: Website Content Ingestion and Vector Indexing

## Overview
This document defines the API contracts for the Docusaurus content ingestion and vector indexing system. Since this is primarily an ETL script rather than a web service, these contracts represent the internal function interfaces and expected data flows.

## Function Interfaces

### 1. fetch_all_urls
Discovers and returns all valid URLs from a Docusaurus site.

```python
def fetch_all_urls(base_url: str, max_pages: int = 500) -> List[str]:
    """
    Fetches all discoverable URLs from a Docusaurus site.

    Args:
        base_url (str): The base URL of the Docusaurus site to crawl
        max_pages (int): Maximum number of pages to crawl (default 500)

    Returns:
        List[str]: List of valid URLs discovered on the site

    Raises:
        requests.RequestException: If there's an issue connecting to the site
        ValueError: If the base_url is invalid
    """
    pass
```

**Input Validation**:
- `base_url` must be a valid HTTP/HTTPS URL
- `max_pages` must be a positive integer (1-10000)

**Output Format**:
- Returns a list of unique, valid URLs that are part of the Docusaurus site
- URLs should be absolute, not relative
- Filters out external links, media files, and other non-content URLs

### 2. get_text_from_url
Extracts and cleans text content from a single URL.

```python
def get_text_from_url(url: str) -> Dict[str, Any]:
    """
    Extracts and cleans text content from a given URL.

    Args:
        url (str): The URL to extract content from

    Returns:
        Dict[str, Any]: Dictionary containing extracted content and metadata
        {
            "url": str,           # Source URL
            "title": str,         # Page title
            "text": str,          # Cleaned text content
            "word_count": int,    # Number of words in content
            "content_hash": str,  # Hash of the content for deduplication
            "metadata": dict      # Additional page metadata
        }

    Raises:
        requests.RequestException: If there's an issue connecting to the URL
        ValueError: If the URL is invalid or content extraction fails
    """
    pass
```

**Input Validation**:
- `url` must be a valid HTTP/HTTPS URL
- Function should handle various content types gracefully

**Output Format**:
- Returns dictionary with content and metadata
- `text` field contains cleaned content without HTML tags, navigation, or footer
- `content_hash` is SHA256 hash of the cleaned text content

### 3. split_text
Splits a large text into smaller, semantically coherent chunks.

```python
def split_text(text: str, max_chunk_size: int = 4000, overlap: int = 200) -> List[Dict[str, Any]]:
    """
    Splits text into smaller chunks while preserving semantic coherence.

    Args:
        text (str): The text to split
        max_chunk_size (int): Maximum size of each chunk in characters (default 4000)
        overlap (int): Number of characters to overlap between chunks (default 200)

    Returns:
        List[Dict[str, Any]]: List of chunk dictionaries
        [
            {
                "text": str,          # The chunk text
                "chunk_index": int,   # Position of this chunk in the original text
                "word_count": int,    # Number of words in this chunk
                "content_hash": str   # Hash of this chunk content
            }
        ]

    Raises:
        ValueError: If text is empty or chunk parameters are invalid
    """
    pass
```

**Input Validation**:
- `text` must not be empty
- `max_chunk_size` must be positive
- `overlap` must be non-negative and less than `max_chunk_size`

**Output Format**:
- Returns list of chunk dictionaries
- Each chunk maintains semantic coherence
- Chunks may overlap to preserve context across boundaries

### 4. generate_embeddings
Converts text chunks into vector embeddings using Cohere API.

```python
def generate_embeddings(chunks: List[Dict[str, Any]], model: str = "embed-multilingual-v3.0") -> List[Dict[str, Any]]:
    """
    Generates vector embeddings for text chunks using Cohere API.

    Args:
        chunks (List[Dict[str, Any]]): List of text chunks to embed
        model (str): Cohere model to use for embeddings (default "embed-multilingual-v3.0")

    Returns:
        List[Dict[str, Any]]: List of embedding dictionaries
        [
            {
                "chunk_id": str,      # ID from the input chunk
                "vector": List[float], # The embedding vector
                "text": str,          # Original text (for reference)
                "content_hash": str   # Content hash for verification
            }
        ]

    Raises:
        cohere.CohereError: If there's an issue with the Cohere API
        ValueError: If chunks are invalid or model is unsupported
    """
    pass
```

**Input Validation**:
- `chunks` must be a non-empty list
- Each chunk must have required fields (text, chunk_id)
- `model` must be a valid Cohere embedding model

**Output Format**:
- Returns list of embedding dictionaries
- Vector dimensions depend on the chosen model (typically 1024 for embed-multilingual-v3.0)
- Maintains correspondence with input chunks

### 5. create_rag_collection
Creates or initializes the Qdrant collection for storing embeddings.

```python
def create_rag_collection(collection_name: str = "book_embeddings", vector_size: int = 1024) -> bool:
    """
    Creates or ensures existence of the Qdrant collection for embeddings.

    Args:
        collection_name (str): Name of the collection (default "book_embeddings")
        vector_size (int): Dimension of the embedding vectors (default 1024)

    Returns:
        bool: True if collection exists or was created successfully

    Raises:
        qdrant_client.exceptions.ResponseHandlingException: If there's an issue with Qdrant
    """
    pass
```

**Input Validation**:
- `collection_name` must be a valid Qdrant collection name
- `vector_size` must be positive and match the embedding model

**Output Format**:
- Returns boolean indicating success
- Creates collection with appropriate vector size and configuration

### 6. store_chunks_in_qdrant
Stores vector embeddings in Qdrant with metadata.

```python
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
        {
            "total_processed": int,   # Number of embeddings processed
            "successful": int,       # Number successfully stored
            "failed": int,           # Number that failed to store
            "errors": List[str]      # List of error messages for failed items
        }

    Raises:
        qdrant_client.exceptions.ResponseHandlingException: If there's an issue with Qdrant
        ValueError: If embeddings are invalid
    """
    pass
```

**Input Validation**:
- `embeddings` must be a non-empty list of valid embedding dictionaries
- Each embedding must have required fields (vector, chunk_id, metadata)
- `collection_name` must refer to an existing collection

**Output Format**:
- Returns summary dictionary with processing results
- Tracks successful and failed operations separately

## Data Contracts

### Content Extraction Contract
The system expects Docusaurus sites to have:
- Semantic HTML with proper heading hierarchy
- Main content within `<main>`, `<article>`, or similar semantic elements
- Navigation elements that can be reliably excluded from content
- Consistent URL patterns for site navigation

### Embedding Quality Contract
The system guarantees:
- Embeddings preserve semantic meaning of source content
- Consistent vector dimensions across all embeddings
- Proper handling of special characters and multilingual content
- Rate limiting to stay within API usage limits

### Storage Contract
The system ensures:
- Each embedding is stored with complete metadata
- Duplicate content is detected and prevented
- Data integrity is maintained during storage operations
- Efficient retrieval through proper indexing

## Error Handling Contracts

### Network Errors
- Implement exponential backoff for retry logic
- Log failed requests with sufficient context for debugging
- Continue processing other items when individual requests fail

### API Limitations
- Handle rate limiting gracefully with appropriate delays
- Batch operations to minimize API calls where possible
- Cache results to avoid redundant API calls

### Data Validation
- Validate content quality before processing
- Reject malformed or empty content early in the pipeline
- Provide clear error messages for debugging