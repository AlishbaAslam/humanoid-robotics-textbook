# Research: Website Content Ingestion and Vector Indexing for RAG Chatbot

## Overview
This research document outlines the technologies, approaches, and best practices for implementing a system that crawls Docusaurus book content, extracts text, generates embeddings using Cohere, and stores them in Qdrant Cloud.

## Technology Investigation

### 1. Web Crawling and Content Extraction

#### Decision: Use requests + BeautifulSoup for content extraction
- **Rationale**: The combination of requests for HTTP operations and BeautifulSoup for HTML parsing is the most reliable and widely-used approach for web scraping in Python. It's lightweight, well-documented, and handles various HTML structures effectively.
- **Alternatives considered**:
  - Selenium: More complex, slower, requires browser instance - overkill for static Docusaurus sites
  - Scrapy: Full-featured framework but more complex than needed for this use case
  - Playwright: Modern but adds unnecessary complexity for static content extraction

#### Docusaurus-Specific Considerations
- Docusaurus sites typically have predictable DOM structures with content in main article containers
- Navigation follows consistent patterns (sidebar, breadcrumbs)
- Need to respect robots.txt and implement rate limiting
- Static content extraction is sufficient (no JavaScript rendering needed)

### 2. Text Processing and Cleaning

#### Decision: Use BeautifulSoup + custom cleaning functions
- **Rationale**: BeautifulSoup provides excellent tools for removing HTML tags and extracting clean text. Custom cleaning functions can handle Docusaurus-specific elements like navigation, headers, and footers.
- **Alternatives considered**:
  - Newspaper3k: Geared towards news articles, not documentation sites
  - Trafilatura: Good for generic web content but less control over Docusaurus-specific elements
  - Raw regex: Less reliable and harder to maintain

#### Text Chunking Approach
- **Decision**: RecursiveCharacterTextSplitter or custom implementation
- **Rationale**: Split content by character boundaries while maintaining semantic coherence. Prioritize splitting on punctuation marks, sentences, paragraphs, etc.
- **Chunk size**: Target 200-500 words per chunk to balance context and efficiency
- **Overlap**: 20-50 words overlap to maintain context across chunks

### 3. Embedding Generation

#### Decision: Use Cohere's embed-multilingual-v3.0 model
- **Rationale**: Cohere's embedding models offer high quality semantic representations, good multilingual support, and reliable performance. The v3.0 model provides the best balance of cost and performance.
- **Alternatives considered**:
  - OpenAI embeddings: Higher cost, potentially less semantic understanding
  - Hugging Face models: Self-hosted option but adds infrastructure complexity
  - Sentence Transformers: Local option but requires more maintenance

#### Rate Limiting and Cost Optimization
- Batch API calls to minimize requests
- Cache embeddings to avoid redundant API calls
- Implement exponential backoff for API failures
- Monitor token usage to stay within budget

### 4. Vector Storage

#### Decision: Use Qdrant Cloud with the book_embeddings collection
- **Rationale**: Qdrant is designed for vector similarity search, offers excellent performance, and integrates well with Python. The cloud version provides reliability and scalability without infrastructure management.
- **Alternatives considered**:
  - Pinecone: Proprietary, different pricing model
  - Weaviate: Good alternative but Qdrant has simpler Python integration
  - FAISS: Local-only, no cloud managed option
  - Elasticsearch: Not optimized for vector search

#### Collection Design
- **Collection name**: book_embeddings
- **Vector dimensions**: Match Cohere model output (likely 1024 for embed-multilingual-v3.0)
- **Payload structure**: Include metadata like URL, title, chunk index, source document info

### 5. Architecture Patterns

#### Decision: Single-file script with modular functions
- **Rationale**: The requirements specify a single file (main.py) with specific functions, which is appropriate for this ETL-style pipeline. Modular functions promote testability and maintainability.
- **Function breakdown**:
  - `fetch_all_urls`: Discover all pages from the Docusaurus site
  - `get_text_from_url`: Extract and clean content from a single URL
  - `split_text`: Split content into appropriately sized chunks
  - `generate_embeddings`: Convert text chunks to vector embeddings
  - `create_rag_collection`: Initialize the Qdrant collection
  - `store_chunks_in_qdrant`: Store embeddings with metadata
  - `main`: Coordinate the entire workflow

## Best Practices Identified

### Error Handling
- Implement retry mechanisms for network requests and API calls
- Log errors with sufficient context for debugging
- Continue processing when individual URLs fail
- Validate content quality before processing

### Performance Optimization
- Use concurrent requests for faster crawling (with rate limiting)
- Batch embedding API calls to reduce latency
- Implement progress tracking for long-running processes
- Cache intermediate results where appropriate

### Data Quality
- Remove boilerplate content (navigation, footer, header)
- Preserve semantic structure and meaning
- Handle different content types (text, code blocks, lists)
- Verify content uniqueness to prevent duplicates

## Security and Compliance
- Store API keys in environment variables (not in code)
- Respect robots.txt and implement appropriate delays
- Validate URLs to prevent access to unintended resources
- Implement proper authentication for Qdrant Cloud

## Implementation Considerations

### URL Discovery Strategy
For Docusaurus sites, URLs can be discovered through:
1. Sitemap.xml if available
2. Following internal links recursively from the homepage
3. Parsing the sidebar navigation structure
4. Using the Docusaurus-generated routes configuration

### Content Extraction Strategy
Docusaurus sites typically have:
- Main content in `<article>` or `<main>` tags
- Page titles in `<h1>` tags
- Breadcrumbs and navigation elements to exclude
- Code blocks and other structured content to preserve appropriately

### Metadata Strategy
Store with each embedding:
- Source URL
- Page title
- Chunk sequence number
- Original content length
- Timestamp of ingestion
- Content hash for deduplication