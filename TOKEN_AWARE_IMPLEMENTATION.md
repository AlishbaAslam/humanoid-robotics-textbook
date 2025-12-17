# Token-Aware, Resumable Embedding Pipeline Implementation

## Overview

This implementation provides a sophisticated embedding generation pipeline that incorporates token budgeting, adaptive batch sizing, progress persistence, and enforced cooldown windows to safely handle Cohere API rate limits while allowing partial progress and graceful resumption.

## Key Features Implemented

### 1. Token Budgeting & Estimation
- **Token Estimation**: `estimate_token_count()` uses a 1 token ≈ 4 characters approximation
- **Budget Management**: `TokenBudgetManager` tracks usage within time windows (1-minute periods)
- **Configurable Limits**: `TOKEN_BUDGET_PER_MINUTE` (default 100,000 tokens) with buffer (5,000 tokens)

### 2. Adaptive Batch Sizing
- **Dynamic Sizing**: `calculate_adaptive_batch_size()` adjusts batch size based on remaining token budget
- **Minimum Threshold**: `MIN_TOKENS_PER_REQUEST` (1,000 tokens) ensures efficiency
- **Smart Scaling**: Reduces batch size when approaching budget limits

### 3. Progress Persistence & Resume
- **State Tracking**: `save_progress()` and `load_progress()` functions maintain pipeline state
- **Checkpointing**: Progress saved after each successful batch
- **Resume Capability**: Automatically resumes from last completed chunk
- **Progress File**: `embedding_progress.json` stores state between runs

### 4. Enforced Cooldown Windows
- **Rate Limit Handling**: Detects HTTP 429 and rate limit errors
- **Exponential Backoff**: With jitter to prevent thundering herd problems
- **Maximum Limits**: `MAX_COOLDOWN_WINDOW` (300 seconds) prevents excessive waits
- **Graceful Failure**: Pipeline stops and preserves state when rate limits are exceeded

### 5. Enhanced Error Handling
- **Partial Progress**: Continues processing after non-rate-limit errors
- **Graceful Degradation**: Handles various error conditions appropriately
- **Comprehensive Logging**: Detailed logs for monitoring and debugging

## Architecture

```
Input Chunks
    ↓
Adaptive Batch Sizing (based on token budget)
    ↓
Token Budget Validation
    ↓
Rate-Limited API Calls (with retry logic)
    ↓
Progress Persistence (after each batch)
    ↓
Output Embeddings
```

## Configuration Constants

```python
TOKEN_BUDGET_PER_MINUTE = 100000    # Maximum tokens per minute
TOKEN_BUFFER = 5000                 # Safety buffer below limit
MIN_TOKENS_PER_REQUEST = 1000       # Minimum efficient batch size
MAX_COOLDOWN_WINDOW = 300           # Maximum wait time in seconds
PROGRESS_FILE = "embedding_progress.json"  # State persistence
```

## Resumable Pipeline Behavior

1. **Start**: Checks for existing progress file
2. **Resume**: If found, starts from the last completed chunk
3. **Process**: Continues embedding remaining chunks
4. **Persist**: Saves progress after each successful batch
5. **Complete**: Removes progress file on successful completion
6. **Interrupt**: Maintains progress file on failure for later resumption

## Token Window Management

- **Time Windows**: 1-minute intervals for token budgeting
- **Automatic Reset**: Budget window resets after 60 seconds of elapsed time
- **Wait Logic**: Pauses when budget is exceeded until next window
- **Efficiency Check**: Prevents requests below minimum token threshold

## Integration

The implementation is backward compatible - existing code calling `generate_embeddings()` will automatically benefit from all new features without requiring code changes.