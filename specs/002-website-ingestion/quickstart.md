# Quickstart Guide: Website Content Ingestion and Vector Indexing

## Overview
This guide provides instructions to quickly set up and run the Docusaurus content ingestion system that extracts content, generates embeddings using Cohere, and stores them in Qdrant Cloud.

## Prerequisites
- Python 3.11 or higher
- Access to Cohere API (sign up at [cohere.ai](https://cohere.ai))
- Qdrant Cloud account (sign up at [qdrant.tech](https://qdrant.tech))
- The target Docusaurus site must be publicly accessible

## Setup

### 1. Clone and Navigate to Backend Directory
```bash
cd /path/to/your/project/backend
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables
Create a `.env` file based on the example:

```bash
cp .env.example .env
```

Edit the `.env` file with your actual credentials:
```bash
COHERE_API_KEY=your_actual_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
```

## Usage

### Run the Complete Ingestion Pipeline
```bash
python main.py
```

The script will:
1. Crawl the specified Docusaurus site
2. Extract and clean content from all pages
3. Split content into semantic chunks
4. Generate embeddings using Cohere
5. Create the "book_embeddings" collection in Qdrant
6. Store all embeddings with metadata

### Run with Custom Parameters
```bash
python main.py --url "https://your-docusaurus-site.com" --collection "custom_collection_name" --chunk-size 3000
```

## Configuration Options

### Command Line Arguments
- `--url`: Base URL of the Docusaurus site to crawl (default: from environment or hardcoded)
- `--collection`: Qdrant collection name (default: "book_embeddings")
- `--chunk-size`: Maximum character size for text chunks (default: 4000)
- `--overlap`: Character overlap between chunks (default: 200)
- `--max-pages`: Maximum number of pages to crawl (default: 500)

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL of your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for your Qdrant Cloud instance
- `BASE_URL`: Default URL to crawl if not specified in command line

## Verification

### Check Qdrant Collection
After running the script, verify that:
1. The "book_embeddings" collection exists in your Qdrant Cloud instance
2. The collection contains the expected number of vectors
3. Metadata includes source URLs and content information

### Sample Verification Script
```python
from qdrant_client import QdrantClient

client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
collection_info = client.get_collection("book_embeddings")
print(f"Points in collection: {collection_info.points_count}")
```

## Troubleshooting

### Common Issues

1. **API Rate Limits**: If you encounter rate limit errors, the system implements exponential backoff automatically.

2. **Connection Errors**: Ensure your Qdrant URL and API key are correct and that your network allows outbound connections.

3. **Empty Results**: Verify that the target Docusaurus site is accessible and that the URL is correctly formatted.

4. **Memory Issues**: For very large sites, consider increasing the chunk size or limiting the number of pages crawled.

### Logging
The system logs progress and errors to standard output. For detailed logging, run with the `--verbose` flag:
```bash
python main.py --verbose
```

## Next Steps

1. Integrate with your RAG chatbot to query the stored embeddings
2. Set up scheduled runs for content updates
3. Monitor embedding quality and collection growth
4. Implement content update detection to avoid reprocessing unchanged content