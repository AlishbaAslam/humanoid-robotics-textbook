# Docusaurus Content Ingestion Pipeline

This project implements a content ingestion pipeline that crawls Docusaurus sites, extracts content, generates embeddings using Cohere, and stores them in Qdrant for RAG (Retrieval Augmented Generation) applications.

## Overview

The system performs the following steps:
1. Crawls a Docusaurus site to discover all content pages
2. Extracts and cleans text content from each page
3. Splits content into semantically coherent chunks
4. Generates vector embeddings using Cohere's API
5. Stores embeddings in Qdrant with associated metadata

## Prerequisites

- Python 3.11 or higher
- Access to Cohere API (sign up at [cohere.ai](https://cohere.ai))
- Qdrant Cloud account (sign up at [qdrant.tech](https://qdrant.tech))

## Setup

1. Clone the repository and navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Configure environment variables:
   ```bash
   cp .env.example .env
   ```

5. Edit the `.env` file with your actual credentials:
   ```bash
   COHERE_API_KEY=your_actual_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   BASE_URL=https://your-docusaurus-site.com
   ```

## Usage

### Run the Complete Ingestion Pipeline

```bash
python main.py
```

The script will crawl the site specified in BASE_URL environment variable and process all pages.

### Run with Custom Parameters

```bash
python main.py --url "https://your-docusaurus-site.com" --collection "custom_collection_name" --chunk-size 3000 --max-pages 100
```

### Command Line Arguments

- `--url`: Base URL of the Docusaurus site to crawl (overrides environment variable)
- `--collection`: Qdrant collection name (default: "book_embeddings")
- `--chunk-size`: Maximum character size for text chunks (default: 4000)
- `--overlap`: Character overlap between chunks (default: 200)
- `--max-pages`: Maximum number of pages to crawl (default: 500)

## Functions

The main module contains the following key functions:

- `fetch_all_urls(base_url, max_pages)`: Discovers all pages from a Docusaurus site
- `get_text_from_url(url)`: Extracts and cleans content from a single URL
- `split_text(text, max_chunk_size, overlap)`: Splits content into chunks
- `generate_embeddings(chunks, model)`: Creates vector embeddings using Cohere
- `create_rag_collection(collection_name, vector_size)`: Sets up Qdrant collection
- `store_chunks_in_qdrant(embeddings, collection_name)`: Stores embeddings in Qdrant
- `main()`: Orchestrates the complete workflow

## Testing

Run the tests using pytest:

```bash
pytest tests/
```

## Architecture

The system is designed as a single-file script (main.py) with modular functions for:

1. **Content Discovery**: Crawling and URL discovery
2. **Content Extraction**: HTML parsing and text extraction
3. **Content Processing**: Cleaning and chunking
4. **Embedding Generation**: Vector creation using Cohere
5. **Vector Storage**: Qdrant integration for storage and retrieval

## Error Handling

The system includes comprehensive error handling:
- Network request retries with exponential backoff
- Graceful degradation when individual pages fail
- Validation of content quality before processing
- Rate limiting for API calls

## Performance Considerations

- Batch processing for embedding generation
- Concurrent requests for faster crawling (with respect for server limits)
- Efficient chunking algorithm to maintain semantic coherence
- Duplicate detection to prevent redundant storage