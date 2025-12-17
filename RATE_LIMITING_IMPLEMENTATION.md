# Rate-Limit-Safe Embedding Generation Pipeline

## Overview

This implementation provides a robust embedding generation pipeline that safely handles Cohere API rate limits with proper throttling, retry logic, and exponential backoff. The solution ensures that the system can handle HTTP 429 errors gracefully while allowing partial progress and preventing pipeline failure.

## Key Features

### 1. Rate Limiting & Throttling
- **Batch Size Control**: Uses `EMBEDDING_BATCH_SIZE = 96` to respect Cohere's API limits
- **Inter-batch Delays**: Adds `RATE_LIMIT_DELAY = 1.0s` between batches to prevent overwhelming the API
- **Configurable Constants**: All rate limiting parameters are defined as constants for easy adjustment

### 2. Retry Logic with Exponential Backoff
- **Maximum Retry Attempts**: Configured with `MAX_RETRY_ATTEMPTS = 5` to prevent infinite loops
- **Exponential Backoff**: Uses `INITIAL_BACKOFF = 1.0s` with exponential increase (2^attempt)
- **Jitter**: Adds randomization to prevent thundering herd problems
- **Maximum Cap**: Respects `MAX_BACKOFF = 60.0s` to prevent excessively long waits

### 3. Smart Error Handling
- **Rate Limit Detection**: Identifies rate limit errors from error messages containing "rate limit", "429", or "too many requests"
- **Non-Rate Limit Errors**: Immediately raises non-rate limit errors without retry
- **Proper Logging**: Warns when rate limits are hit and logs retry attempts

## Implementation Details

### Core Functions

1. **`calculate_exponential_backoff(attempt)`**:
   - Calculates backoff time using exponential formula: `initial_backoff * (2^attempt)`
   - Applies jitter to prevent synchronized retries
   - Respects maximum backoff time

2. **`make_rate_limited_embedding_request(cohere_client, texts, model, input_type)`**:
   - Handles all embedding requests with rate limiting
   - Implements retry logic with exponential backoff
   - Distinguishes between rate limit and other errors

3. **Enhanced Embedding Functions**:
   - `generate_query_embedding()`: Updated to use rate-limited approach
   - `generate_embeddings()`: Updated to use rate-limited approach with inter-batch delays

### Constants

```python
EMBEDDING_BATCH_SIZE = 96  # Maximum number of texts per Cohere API request
RATE_LIMIT_DELAY = 1.0     # Base delay in seconds between requests
MAX_RETRY_ATTEMPTS = 5     # Maximum number of retry attempts for rate limit errors
INITIAL_BACKOFF = 1.0      # Initial backoff time in seconds
MAX_BACKOFF = 60.0         # Maximum backoff time in seconds
```

## Benefits

- **Resilience**: Pipeline continues to make progress even when rate limits are encountered
- **API Respect**: Properly handles rate limits to maintain good standing with Cohere API
- **Partial Progress**: Allows partial completion when some requests fail
- **Performance**: Optimized batching and throttling for efficient API usage
- **Maintainability**: Clear, configurable implementation that's easy to adjust

## Usage

The implementation is backward compatible - existing code calling `generate_query_embedding()` and `generate_embeddings()` will automatically benefit from the rate limiting and retry logic without any code changes required.