# Quickstart Guide: Agent Development with Retrieval Integration

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Docker (optional, for containerization)
- Qdrant Cloud account (or local Qdrant instance)
- OpenAI API key
- Cohere API key (for embeddings)

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create virtual environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

### 4. Set up environment variables
Create a `.env` file based on `.env.example`:
```bash
cp .env.example .env
```

Update the `.env` file with your API keys and configuration:
```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=humanoid_robotics_content
```

## Running the Application

### 1. Start the development server
```bash
cd backend
python -m src.api.main
```

### 2. Or using uvicorn (recommended for development)
```bash
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### 3. Or using Docker
```bash
docker-compose up --build
```

## Testing the API

### 1. Health check
```bash
curl http://localhost:8000/health
```

### 2. Submit a query
```bash
curl -X POST http://localhost:8000/api/agent/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_token_if_required" \
  -d '{
    "query": "What are the key components of a humanoid robot?",
    "session_id": "test_session_1"
  }'
```

## Key Components

### Agent Structure
```
backend/
├── src/
│   ├── agents/           # Agent implementations
│   ├── api/             # FastAPI application and routes
│   ├── services/        # Business logic services
│   ├── models/          # Data models
│   ├── config/          # Configuration
│   └── utils/           # Utility functions
```

### Main Modules
- `src.agents.openai_agent.py`: OpenAI Agent implementation
- `src.services.qdrant_service.py`: Qdrant database interactions
- `src.services.retrieval_service.py`: Content retrieval logic
- `src.api.routes.agent.py`: API endpoints for agent queries

## Configuration

### Qdrant Setup
The application expects a Qdrant collection with humanoid robotics content. To set up:

1. Create a collection in Qdrant with appropriate vector size (based on embedding model)
2. Index your book content using the embedding service
3. Ensure the collection name matches the configuration

### API Endpoints
- `POST /api/agent/query` - Submit a single query
- `POST /api/agent/query/batch` - Submit multiple queries
- `GET /health` - Health check

## Development

### Running Tests
```bash
cd backend
pytest tests/
```

### Code Formatting
```bash
# Format code with black
black src/

# Check for linting issues
flake8 src/
```

## Troubleshooting

### Common Issues
1. **API Keys**: Ensure all required API keys are set in environment variables
2. **Qdrant Connection**: Verify Qdrant URL and API key are correct
3. **Embedding Mismatch**: Ensure embedding dimensions match between query and stored content

### Logging
The application logs to stdout by default. For more detailed logging, set the LOG_LEVEL environment variable to "DEBUG".