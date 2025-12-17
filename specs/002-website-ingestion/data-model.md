# Data Model: Website Content Ingestion and Vector Indexing

## Overview
This document defines the data structures and relationships for the Docusaurus content ingestion and vector indexing system.

## Core Entities

### 1. ContentChunk
Represents a segment of cleaned text extracted from a Docusaurus page.

```python
class ContentChunk:
    id: str                    # Unique identifier (UUID or content hash)
    text: str                  # The actual text content of the chunk
    source_url: str           # Original URL where this content came from
    title: str                # Title of the source page
    chunk_index: int          # Sequential number of this chunk within the source
    total_chunks: int         # Total number of chunks from this source
    content_hash: str         # Hash of the content for deduplication
    created_at: datetime      # Timestamp when this chunk was created
    word_count: int           # Number of words in the chunk
    source_metadata: dict     # Additional metadata from the source page
```

**Validation Rules**:
- `text` must be between 50 and 1000 words (approximately 250-5000 characters)
- `source_url` must be a valid URL
- `chunk_index` must be >= 0
- `total_chunks` must be > 0 and >= `chunk_index`
- `word_count` must be > 0

### 2. VectorEmbedding
Represents the vector representation of a content chunk with associated metadata.

```python
class VectorEmbedding:
    point_id: str             # Unique identifier in Qdrant (matches ContentChunk.id)
    vector: List[float]       # The actual embedding vector (dimension varies by model)
    payload: dict             # Metadata stored alongside the vector
    created_at: datetime      # Timestamp when this embedding was created
```

**Payload Structure**:
```python
{
    "source_url": str,        # Original URL
    "title": str,             # Page title
    "chunk_index": int,       # Position in source document
    "content_hash": str,      # For deduplication
    "word_count": int,        # Length of original content
    "created_at": str,        # ISO timestamp
    "source_title": str,      # Title of the source document
    "content_type": str       # Type of content (text, code, etc.)
}
```

**Validation Rules**:
- `vector` must have consistent dimension (e.g., 1024 for Cohere embed-multilingual-v3.0)
- `payload` must contain required fields as defined above
- `point_id` must be unique within the collection

### 3. SourceDocument
Represents the original Docusaurus page from which content was extracted.

```python
class SourceDocument:
    url: str                  # The canonical URL of the document
    title: str                # The title of the document
    content_length: int       # Total length of extracted content in characters
    word_count: int           # Total number of words in the document
    chunk_count: int          # Number of chunks this document was split into
    extracted_at: datetime    # When the content was extracted
    content_hash: str         # Hash of the full content for change detection
    metadata: dict            # Additional metadata (author, last modified, etc.)
```

**Validation Rules**:
- `url` must be a valid URL
- `title` must not be empty
- `content_length` and `word_count` must be >= 0
- `chunk_count` must be > 0

## Relationships

### ContentChunk → SourceDocument
- One-to-many relationship: One source document produces multiple content chunks
- `ContentChunk.source_url` references `SourceDocument.url`
- `ContentChunk.chunk_index` indicates position within the source document (0-indexed)

### VectorEmbedding → ContentChunk
- One-to-one relationship: Each content chunk corresponds to one vector embedding
- `VectorEmbedding.point_id` matches `ContentChunk.id`
- Both represent the same semantic content in different formats

## Qdrant Collection Schema

### Collection: book_embeddings
- **Vector Configuration**:
  - Size: 1024 (for Cohere embed-multilingual-v3.0)
  - Distance: Cosine (optimal for semantic similarity)

- **Payload Schema**:
  - Indexed Fields:
    - `source_url` (keyword, indexed for fast lookup)
    - `content_hash` (keyword, indexed for deduplication)
    - `chunk_index` (integer, indexed for ordering)
    - `created_at` (datetime, indexed for temporal queries)

- **Payload Example**:
```json
{
  "source_url": "https://alishbaaslam.github.io/humanoid-robotics-textbook/introduction",
  "title": "Introduction to Humanoid Robotics",
  "chunk_index": 0,
  "content_hash": "a1b2c3d4e5f6...",
  "word_count": 420,
  "created_at": "2025-12-16T10:30:00Z",
  "source_title": "Humanoid Robotics Textbook",
  "content_type": "text"
}
```

## Data Flow

1. **Extraction**: SourceDocument objects are created from crawled Docusaurus pages
2. **Processing**: SourceDocument content is split into ContentChunk objects
3. **Embedding**: ContentChunk objects are converted to VectorEmbedding objects
4. **Storage**: VectorEmbedding objects are stored in Qdrant with payload metadata

## Quality Assurance

### Deduplication Strategy
- Calculate content hash for each ContentChunk
- Query Qdrant for existing content with the same hash before storing
- Skip storage if duplicate is detected

### Integrity Checks
- Verify that all referenced source URLs exist in the system
- Ensure chunk sequences are complete for each source document
- Validate that embedding dimensions match expected values
- Confirm that payload metadata is complete and properly formatted

## Performance Considerations

### Indexing Strategy
- Create indexes on frequently queried fields (source_url, content_hash)
- Optimize for vector similarity search performance
- Consider partitioning for very large collections

### Storage Efficiency
- Compress large text fields where possible
- Use efficient data types for numeric fields
- Minimize payload size while preserving necessary metadata