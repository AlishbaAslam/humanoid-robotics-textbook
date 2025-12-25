# API Documentation: Agent Development with Retrieval Integration

## Overview
This document describes the API endpoints and integration patterns for the RAG (Retrieval-Augmented Generation) chatbot system that allows users to ask questions about humanoid robotics content.

## Backend API Endpoints

### Agent Query Endpoint
- **URL**: `/api/agent/query`
- **Method**: `POST`
- **Description**: Submit a query to the backend agent for processing with RAG functionality

#### Request Body
```json
{
  "query": "What are the key components of a humanoid robot?",
  "session_id": "optional-session-identifier",
  "user_id": "optional-user-identifier"
}
```

#### Response
```json
{
  "id": "response-id",
  "query": "What are the key components of a humanoid robot?",
  "response": "The key components include...",
  "retrieved_chunks": [...],
  "confidence": 0.85,
  "sources": ["Chapter 2: Components", "Chapter 3: Design"],
  "timestamp": "2025-12-17T10:30:00Z"
}
```

#### Error Response
```json
{
  "error": "QUERY_PROCESSING_ERROR",
  "message": "Failed to process query due to internal error",
  "details": {}
}
```

### Batch Query Endpoint
- **URL**: `/api/agent/query/batch`
- **Method**: `POST`
- **Description**: Submit multiple queries to the backend agent in a single request

### Session Management Endpoints
- **Get Session**: `GET /api/agent/session/{session_id}`
- **Clear Session**: `DELETE /api/agent/session/{session_id}`

### Health Check Endpoint
- **URL**: `/health`
- **Method**: `GET`
- **Description**: Check the health status of the agent service

## Frontend API Service

### submitQuery(queryData)
Submits a single query to the backend agent.

```javascript
import { submitQuery } from './api/agentService';

const response = await submitQuery({
  query: "What are the key components?",
  session_id: "session-123"
});
```

### submitBatchQuery(batchData)
Submits multiple queries to the backend agent.

```javascript
import { submitBatchQuery } from './api/agentService';

const response = await submitBatchQuery({
  queries: [
    { query: "Query 1" },
    { query: "Query 2" }
  ],
  session_id: "session-123"
});
```

### checkHealth()
Checks the health status of the backend service.

```javascript
import { checkHealth } from './api/agentService';

const healthStatus = await checkHealth();
```

## Environment Configuration

### Backend (.env)
```
GEMINI_API_KEY=your-gemini-api-key
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=your-qdrant-url
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_embeddings
```

### Frontend (.env)
```
REACT_APP_API_BASE_URL=http://localhost:8000
```

## Error Handling

### Frontend Error Handling
- Network errors: Display user-friendly message and retry option
- Backend errors: Show appropriate error messages from API response
- Validation errors: Validate input before sending to backend

### Backend Error Handling
- API key validation
- Qdrant connection errors
- Agent processing errors
- Proper error response formatting

## Testing

### Unit Tests
- Test individual API service functions
- Test chat component state management
- Test error handling utilities

### Integration Tests
- Test frontend-backend communication flow
- Test full query-response cycle
- Test error scenarios and handling

### End-to-End Tests
- Test complete user journey from query submission to response display
- Test error scenarios end-to-end
- Test session management functionality