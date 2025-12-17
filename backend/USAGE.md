# Backend Usage Guide

This document explains how to use the backend services for the humanoid robotics textbook project.

## Prerequisites

Before running any scripts, make sure you have the required environment variables set up in a `.env` file:

```bash
# Qdrant Vector Database
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_embeddings

# AI/ML Services
COHERE_API_KEY=your_cohere_api_key
OPENAI_API_KEY=your_openai_api_key  # Optional, for OpenAI agent
GEMINI_API_KEY=your_gemini_api_key  # Optional, for Gemini agent

# Application settings
HOST=0.0.0.0
PORT=8000
```

## Setting up the Vector Database

### 1. Create the Qdrant Collection

First, ensure your Qdrant collection exists:

```bash
cd backend
python create_collection.py
```

### 2. Ingest Content into the Database

To ingest content from a website (like your Docusaurus documentation):

```bash
cd backend
python ingest_content.py --url https://your-website-url.com
```

Additional options:
- `--max-pages`: Maximum number of pages to crawl (default: 500)
- `--chunk-size`: Maximum character size for text chunks (default: 4000)
- `--overlap`: Character overlap between chunks (default: 200)

Example:
```bash
python ingest_content.py --url https://your-docs-site.com --max-pages 200 --chunk-size 3000
```

## Running the API Server

To start the backend API server:

```bash
cd backend
python -m src.api.main
```

Or using uvicorn directly:
```bash
cd backend
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload
```

## Testing the Setup

### 1. Check API Health
```bash
curl http://localhost:8000/health
```

### 2. Test a Query
```bash
curl -X POST http://localhost:8000/api/agent/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2 middleware architecture?"}'
```

## Troubleshooting

### Common Issues:

1. **Collection not found**: Make sure you've run the ingestion process to create the collection and populate it with data.

2. **API keys not set**: Ensure all required API keys are set in your `.env` file.

3. **Qdrant connection issues**: Verify that your QDRANT_URL and QDRANT_API_KEY are correct.

### Useful Commands:

- Check collection status: `python create_collection.py`
- Verify content exists: Check the number of points in your collection after ingestion
- View API documentation: Visit `http://localhost:8000/docs` when the server is running