# Quickstart: Retrieval Pipeline Testing for RAG Chatbot

## Overview
This guide will help you set up and run the retrieval pipeline testing for the RAG chatbot. The testing system allows you to validate that semantic search returns relevant content from the Qdrant vector store.

## Prerequisites
- Python 3.11 or higher
- Access to Qdrant vector database (with book embeddings stored)
- Cohere API key for embedding generation
- Git for version control

## Setup

### 1. Clone the Repository
```bash
git clone [repository-url]
cd humanoid-robotics-textbook
cd backend
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file in the backend directory with the following:
```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
COLLECTION_NAME=book_embeddings
```

### 4. Verify Qdrant Connection
Ensure your Qdrant collection is accessible and contains the book embeddings.

## Running Retrieval Tests

### Basic Query Test
```bash
python main.py --test-retrieval --query "What is ROS2 middleware architecture?"
```

### Multiple Test Queries
```bash
python main.py --test-retrieval --queries-file test_queries.txt
```

### Consistency Test
```bash
python main.py --test-consistency --query "digital twin simulation" --runs 10
```

### Metadata Validation Test
```bash
python main.py --test-metadata --query "embodied intelligence"
```

### Comprehensive Test Suite
```bash
python main.py --test-all
```

## Understanding Results

The testing output will include:
- **Relevance Score**: How semantically related the results are to your query
- **Metadata Accuracy**: Whether source URLs and other metadata are preserved correctly
- **Response Time**: How long the query took to execute
- **Consistency Score**: For repeated queries, how consistent the results are

## Sample Test Queries
Here are some example queries you can use to test the retrieval system:

- "Explain ROS2 architecture and nodes"
- "How does digital twin simulation work?"
- "What is embodied intelligence?"
- "Describe Isaac Sim integration"
- "Explain humanoid robot path planning"

## Troubleshooting

### Common Issues:

1. **Connection Errors**: Verify your Qdrant URL and API key are correct
2. **No Results**: Ensure the Qdrant collection contains embeddings
3. **Poor Relevance**: Check that the same embedding model was used for queries and stored content

### Verification Steps:
1. Confirm Qdrant collection exists and has data
2. Verify Cohere API key is valid
3. Check that the same embedding model is used for queries and stored content