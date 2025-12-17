# Quickstart Guide: Frontend-Backend Integration for RAG Chatbot

## Overview
This guide explains how to set up and use the frontend-backend integration for the RAG chatbot system that allows users to ask questions about humanoid robotics content.

## Prerequisites
- Node.js 18+ (for frontend development)
- Python 3.11+ (for backend development)
- Git
- API keys for the RAG system (Qdrant, Cohere, OpenAI/Gemini)
- Running backend service with agent endpoints

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up the backend
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
cp .env.example .env
# Update .env with your API keys
```

### 3. Start the backend service
```bash
# From the backend directory
uvicorn src.api.main:app --reload --port 8000
```

### 4. Set up the frontend (Docusaurus)
```bash
cd website  # Assuming Docusaurus is in the website directory
npm install
```

### 5. Configure frontend API endpoints
Update the frontend configuration to point to your backend:
```env
REACT_APP_API_BASE_URL=http://localhost:8000
```

## Running the Application

### 1. Start the backend
```bash
cd backend
source venv/bin/activate
uvicorn src.api.main:app --reload --port 8000
```

### 2. Start the frontend
```bash
cd website
npm start
```

## API Communication

### Making a query
The frontend communicates with the backend using the following endpoint:

```
POST /api/agent/query
```

**Request body:**
```json
{
  "query": "What are the key components of a humanoid robot?",
  "session_id": "unique-session-id",
  "user_id": "optional-user-id"
}
```

**Response:**
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

## Frontend Implementation

### 1. API Service Module
Create a service module to handle API communication:

```javascript
// api/agentService.js
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

export const submitQuery = async (queryData) => {
  const response = await fetch(`${API_BASE_URL}/api/agent/query`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(queryData),
  });

  if (!response.ok) {
    throw new Error(`API request failed: ${response.statusText}`);
  }

  return response.json();
};
```

### 2. Chat Component State Management
```javascript
import { useState } from 'react';
import { submitQuery } from '../api/agentService';

const ChatComponent = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim()) return;

    // Add user message to chat
    const userMessage = { type: 'user', content: inputValue, timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Send query to backend
      const response = await submitQuery({ query: inputValue });

      // Add agent response to chat
      const agentMessage = {
        type: 'agent',
        content: response.response,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, agentMessage]);

      setInputValue('');
    } catch (error) {
      // Add error message to chat
      const errorMessage = {
        type: 'error',
        content: 'Sorry, there was an issue processing your query. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };
};
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

## Testing the Integration

### 1. Verify backend is running
```bash
curl http://localhost:8000/health
```

### 2. Test direct API call
```bash
curl -X POST http://localhost:8000/api/agent/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is a humanoid robot?",
    "session_id": "test-session"
  }'
```

### 3. Verify frontend communication
- Open browser developer tools
- Check Network tab for API requests
- Verify responses are received correctly

## Troubleshooting

### Common Issues
1. **CORS errors**: Ensure backend has proper CORS configuration for frontend domain
2. **API key errors**: Verify all required API keys are set in backend .env file
3. **Connection errors**: Check that backend service is running and accessible
4. **Slow responses**: Verify Qdrant and external API connections

### Environment Configuration
Make sure your environment variables are properly set:
- Backend: API keys for Qdrant, Cohere, OpenAI/Gemini
- Frontend: API base URL pointing to backend service

## Next Steps
- Implement session management for conversation history
- Add loading states and better UX feedback
- Implement error boundaries for better error handling
- Add analytics for tracking user interactions