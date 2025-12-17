/**
 * API service module for agent communication
 * Handles communication between frontend and backend agent endpoints
 */

// Get API base URL from environment or default to local development
// Safely access process.env to avoid "process is not defined" in browser
const getApiBaseUrl = () => {
  if (typeof process !== 'undefined' && process.env) {
    return process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
  }
  // Fallback for environments where process is not defined (browser)
  return 'http://localhost:8000';
};

const API_BASE_URL = getApiBaseUrl();

/**
 * Submit a query to the backend agent
 * @param {Object} queryData - Query data containing the question and optional session info
 * @param {string} queryData.query - The user's query text
 * @param {string} [queryData.session_id] - Optional session identifier
 * @param {string} [queryData.user_id] - Optional user identifier
 * @returns {Promise<Object>} The agent's response
 */
export const submitQuery = async (queryData) => {
  // Import error handling utilities
  const { retryWithBackoff, formatApiError, isRecoverableError } = await import('../utils/errorHandling');

  // Define the API call as a function for retry mechanism
  const apiCall = async () => {
    let response;
    try {
      response = await fetch(`${API_BASE_URL}/api/agent/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(queryData),
      });
    } catch (networkError) {
      // This is likely a network error (Failed to fetch, etc.)
      console.error('Network error in submitQuery:', networkError);
      throw networkError;
    }

    if (!response.ok) {
      const error = await formatApiError(response);
      console.error('API error in submitQuery:', error);
      throw error;
    }

    return response.json();
  };

  try {
    // Use retry mechanism with exponential backoff for network errors
    return await retryWithBackoff(apiCall, 3, 1000);
  } catch (error) {
    // Check if the error is recoverable and provide appropriate message
    if (isRecoverableError(error)) {
      console.warn('Recoverable error in submitQuery:', error);
    }
    // Re-throw with additional context
    throw new Error(`Failed to submit query: ${error.message}`);
  }
};

/**
 * Submit multiple queries to the backend agent
 * @param {Object} batchData - Batch query data
 * @param {Array} batchData.queries - Array of query objects
 * @param {string} [batchData.session_id] - Optional session identifier
 * @returns {Promise<Object>} Batch response containing all query results
 */
export const submitBatchQuery = async (batchData) => {
  // Import error handling utilities
  const { retryWithBackoff, formatApiError, isRecoverableError } = await import('../utils/errorHandling');

  // Define the API call as a function for retry mechanism
  const apiCall = async () => {
    let response;
    try {
      response = await fetch(`${API_BASE_URL}/api/agent/query/batch`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(batchData),
      });
    } catch (networkError) {
      // This is likely a network error (Failed to fetch, etc.)
      console.error('Network error in submitBatchQuery:', networkError);
      throw networkError;
    }

    if (!response.ok) {
      const error = await formatApiError(response);
      console.error('API error in submitBatchQuery:', error);
      throw error;
    }

    return response.json();
  };

  try {
    // Use retry mechanism with exponential backoff for network errors
    return await retryWithBackoff(apiCall, 3, 1000);
  } catch (error) {
    // Check if the error is recoverable and provide appropriate message
    if (isRecoverableError(error)) {
      console.warn('Recoverable error in submitBatchQuery:', error);
    }
    // Re-throw with additional context
    throw new Error(`Failed to submit batch query: ${error.message}`);
  }
};

/**
 * Get session state
 * @param {string} sessionId - The session identifier
 * @returns {Promise<Object>} Session state information
 */
export const getSessionState = async (sessionId) => {
  // Import error handling utilities
  const { retryWithBackoff, formatApiError, isRecoverableError } = await import('../utils/errorHandling');

  // Define the API call as a function for retry mechanism
  const apiCall = async () => {
    let response;
    try {
      response = await fetch(`${API_BASE_URL}/api/agent/session/${sessionId}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
    } catch (networkError) {
      // This is likely a network error (Failed to fetch, etc.)
      console.error('Network error in getSessionState:', networkError);
      throw networkError;
    }

    if (!response.ok) {
      if (response.status === 404) {
        throw new Error('Session not found');
      }
      const error = await formatApiError(response);
      console.error('API error in getSessionState:', error);
      throw error;
    }

    return response.json();
  };

  try {
    // Use retry mechanism with exponential backoff for network errors
    return await retryWithBackoff(apiCall, 2, 1000);
  } catch (error) {
    // Check if the error is recoverable and provide appropriate message
    if (isRecoverableError(error)) {
      console.warn('Recoverable error in getSessionState:', error);
    }
    // Re-throw with additional context
    throw new Error(`Failed to get session state: ${error.message}`);
  }
};

/**
 * Clear session
 * @param {string} sessionId - The session identifier
 * @returns {Promise<Object>} Success message
 */
export const clearSession = async (sessionId) => {
  // Import error handling utilities
  const { retryWithBackoff, formatApiError, isRecoverableError } = await import('../utils/errorHandling');

  // Define the API call as a function for retry mechanism
  const apiCall = async () => {
    let response;
    try {
      response = await fetch(`${API_BASE_URL}/api/agent/session/${sessionId}`, {
        method: 'DELETE',
        headers: {
          'Content-Type': 'application/json',
        },
      });
    } catch (networkError) {
      // This is likely a network error (Failed to fetch, etc.)
      console.error('Network error in clearSession:', networkError);
      throw networkError;
    }

    if (!response.ok) {
      const error = await formatApiError(response);
      console.error('API error in clearSession:', error);
      throw error;
    }

    return response.json();
  };

  try {
    // Use retry mechanism with exponential backoff for network errors
    return await retryWithBackoff(apiCall, 2, 1000);
  } catch (error) {
    // Check if the error is recoverable and provide appropriate message
    if (isRecoverableError(error)) {
      console.warn('Recoverable error in clearSession:', error);
    }
    // Re-throw with additional context
    throw new Error(`Failed to clear session: ${error.message}`);
  }
};

/**
 * Check backend health
 * @returns {Promise<Object>} Health check response
 */
export const checkHealth = async () => {
  // Import error handling utilities
  const { retryWithBackoff, formatApiError, isRecoverableError } = await import('../utils/errorHandling');

  // Define the API call as a function for retry mechanism
  const apiCall = async () => {
    let response;
    try {
      response = await fetch(`${API_BASE_URL}/health`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
    } catch (networkError) {
      // This is likely a network error (Failed to fetch, etc.)
      console.error('Network error in checkHealth:', networkError);
      throw networkError;
    }

    if (!response.ok) {
      const error = await formatApiError(response);
      console.error('API error in checkHealth:', error);
      throw error;
    }

    return response.json();
  };

  try {
    // Use retry mechanism with exponential backoff for network errors
    return await retryWithBackoff(apiCall, 2, 500);
  } catch (error) {
    // Check if the error is recoverable and provide appropriate message
    if (isRecoverableError(error)) {
      console.warn('Recoverable error in checkHealth:', error);
    }
    // Re-throw with additional context
    throw new Error(`Failed to check health: ${error.message}`);
  }
};