# Data Model: Frontend-Backend Integration for RAG Chatbot

## Entities

### User Query
**Description**: Text input from the user requesting information about humanoid robotics content

**Fields**:
- `id` (string, required): Unique identifier for the query
- `text` (string, required): The actual query text from the user (1-1000 characters)
- `timestamp` (datetime, required): When the query was submitted
- `session_id` (string, optional): Session identifier for maintaining conversation context
- `user_id` (string, optional): Identifier for the user making the query

**Validation Rules**:
- `text` must be 1-1000 characters
- `text` must not be empty or whitespace only
- `timestamp` must be within the last 24 hours

### Agent Response
**Description**: The processed response from the backend agent based on the query and RAG content

**Fields**:
- `id` (string, required): Unique identifier for the response
- `query_id` (string, required): Reference to the original query
- `content` (string, required): The agent's response to the user (10-10000 characters)
- `timestamp` (datetime, required): When the response was generated
- `confidence` (float, optional): Confidence level in the response (0.0-1.0)
- `sources` (list, optional): List of source documents referenced
- `error` (object, optional): Error information if response generation failed

**Validation Rules**:
- `content` must be 10-10000 characters
- `confidence` must be between 0.0 and 1.0 if provided
- `sources` must be an array of strings if provided

### Communication Session
**Description**: The connection state between frontend and backend during query/response exchange

**Fields**:
- `id` (string, required): Unique identifier for the session
- `start_time` (datetime, required): When the session was initiated
- `end_time` (datetime, optional): When the session ended
- `status` (string, required): Current status of the session (active, inactive, error)
- `metadata` (object, optional): Additional session information

**Validation Rules**:
- `status` must be one of: "active", "inactive", "error"
- `end_time` must be after `start_time` if provided

## Relationships

### User Query → Agent Response
- One-to-one relationship (each query generates one response)
- User Query is the parent entity
- Agent Response references the query via `query_id`

### Communication Session → User Query
- One-to-many relationship (one session can have multiple queries)
- Communication Session is the parent entity
- User Query references the session via `session_id`

## State Transitions

### Communication Session States
- `ACTIVE`: Session is active and accepting queries
- `INACTIVE`: Session has ended normally
- `ERROR`: Session encountered an error and terminated

### Lifecycle
1. Communication Session enters `ACTIVE` state when user starts interacting
2. User Queries are created within the session context
3. Agent Responses are generated for each query
4. Session transitions to `INACTIVE` when conversation ends or times out
5. Session transitions to `ERROR` if a communication failure occurs

## API Payload Structures

### Query Request
- `query` (string, required): The user's query text
- `session_id` (string, optional): Session identifier to maintain conversation context
- `user_id` (string, optional): User identifier for tracking purposes

### Query Response
- `id` (string, required): Unique identifier for this response
- `query` (string, required): The original query submitted by the user
- `response` (string, required): The agent's response to the query
- `retrieved_chunks` (array, required): List of content chunks used to generate the response
- `confidence` (number, optional): Confidence level in the response (0.0 to 1.0)
- `sources` (array, optional): List of source documents referenced
- `timestamp` (string, required): When the response was generated