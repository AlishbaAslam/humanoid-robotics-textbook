/**
 * API Response Error Handling Utilities
 * Contains utilities for handling different types of API errors in the frontend
 */

/**
 * Determines the type of error from the error object
 * @param {Error|Object|string} error - The error object to categorize
 * @returns {string} The error category ('NETWORK_ERROR', 'SERVER_ERROR', 'CLIENT_ERROR', 'TIMEOUT_ERROR', 'UNKNOWN_ERROR')
 */
export const categorizeError = (error) => {
  if (!error) return 'UNKNOWN_ERROR';

  // Check if it's a network error (fetch failed to reach server)
  if (error.message && (
    error.message.includes('Failed to fetch') ||
    error.message.includes('Network Error') ||
    error.message.includes('TypeError') ||
    error.message.includes('NetworkError')
  )) {
    return 'NETWORK_ERROR';
  }

  // Check if it's a timeout error
  if (error.message && error.message.includes('timeout')) {
    return 'TIMEOUT_ERROR';
  }

  // If error has a status property, categorize based on HTTP status codes
  if (error.status) {
    const status = parseInt(error.status);
    if (status >= 400 && status < 500) {
      return 'CLIENT_ERROR';
    } else if (status >= 500) {
      return 'SERVER_ERROR';
    }
  }

  // If error response has status property
  if (error.response && error.response.status) {
    const status = parseInt(error.response.status);
    if (status >= 400 && status < 500) {
      return 'CLIENT_ERROR';
    } else if (status >= 500) {
      return 'SERVER_ERROR';
    }
  }

  return 'UNKNOWN_ERROR';
};

/**
 * Generates a user-friendly error message based on the error type
 * @param {Error|Object|string} error - The error object
 * @returns {string} A user-friendly error message
 */
export const getUserFriendlyErrorMessage = (error) => {
  if (!error) return 'An unknown error occurred. Please try again.';

  const errorCategory = categorizeError(error);

  switch (errorCategory) {
    case 'NETWORK_ERROR':
      return 'Unable to connect to the server. Please check your internet connection and try again.';

    case 'SERVER_ERROR':
      return 'The server is experiencing issues. Please try again in a moment.';

    case 'CLIENT_ERROR':
      // If we have specific error details from the API
      if (error.response && error.response.data && error.response.data.message) {
        return `Request failed: ${error.response.data.message}`;
      }
      return 'Your request could not be processed. Please check your input and try again.';

    case 'TIMEOUT_ERROR':
      return 'The request took too long to complete. Please try again.';

    default:
      // If error has a message property, use it
      if (error.message) {
        return error.message;
      }

      // If error is a string, return it directly
      if (typeof error === 'string') {
        return error;
      }

      return 'An unexpected error occurred. Please try again.';
  }
};

/**
 * Checks if an error is recoverable (user can take action to resolve it)
 * @param {Error|Object|string} error - The error object
 * @returns {boolean} True if the error is recoverable
 */
export const isRecoverableError = (error) => {
  const errorCategory = categorizeError(error);

  // Network and timeout errors are usually recoverable
  return ['NETWORK_ERROR', 'TIMEOUT_ERROR'].includes(errorCategory);
};

/**
 * Logs error details for debugging (without exposing sensitive information)
 * @param {Error|Object|string} error - The error object
 * @param {string} context - Context where the error occurred
 */
export const logError = (error, context = 'Unknown context') => {
  console.group(`❌ Error in ${context}`);
  console.error('Error object:', error);
  console.error('Error category:', categorizeError(error));
  console.error('User-friendly message:', getUserFriendlyErrorMessage(error));
  console.groupEnd();
};

/**
 * Formats an error response from the API for consistent handling
 * @param {Response} response - The fetch response object
 * @returns {Promise<Object>} A standardized error object
 */
export const formatApiError = async (response) => {
  let errorData = {};

  try {
    // Attempt to parse the response body
    const text = await response.text();
    if (text) {
      try {
        errorData = JSON.parse(text);
      } catch {
        // If parsing fails, use the raw text as the message
        errorData = { message: text };
      }
    }
  } catch (parseError) {
    // If reading the response fails, create a generic error
    errorData = { message: `HTTP Error: ${response.status} ${response.statusText}` };
  }

  // Create a standardized error object
  const error = new Error(errorData.message || `HTTP Error: ${response.status} ${response.statusText}`);
  error.status = response.status;
  error.statusText = response.statusText;
  error.response = response;
  error.data = errorData;

  return error;
};

/**
 * Implements a retry mechanism for API calls with exponential backoff
 * @param {Function} apiCall - The API call function to retry
 * @param {number} maxRetries - Maximum number of retry attempts (default: 3)
 * @param {number} baseDelay - Base delay in milliseconds (default: 1000)
 * @returns {Promise<any>} Result of the API call
 */
export const retryWithBackoff = async (apiCall, maxRetries = 3, baseDelay = 1000) => {
  let lastError;

  for (let attempt = 0; attempt <= maxRetries; attempt++) {
    try {
      return await apiCall();
    } catch (error) {
      lastError = error;

      // Don't retry on client errors (4xx) as they indicate bad requests
      const errorCategory = categorizeError(error);
      if (errorCategory === 'CLIENT_ERROR') {
        throw error;
      }

      // If this was the last attempt, throw the error
      if (attempt === maxRetries) {
        throw error;
      }

      // Calculate delay with exponential backoff and jitter
      const delay = Math.min(baseDelay * Math.pow(2, attempt) + Math.random() * 1000, 10000);

      console.warn(`Attempt ${attempt + 1} failed, retrying in ${delay}ms...`, error.message);
      await new Promise(resolve => setTimeout(resolve, delay));
    }
  }

  throw lastError;
};

/**
 * Validates if an API response is successful
 * @param {Response} response - The fetch response object
 * @returns {boolean} True if the response indicates success
 */
export const isSuccessfulResponse = (response) => {
  return response && response.status >= 200 && response.status < 300;
};

/**
 * Standardizes error handling for API calls
 * @param {Promise<Response>} apiCall - The API call promise
 * @param {string} context - Context for logging
 * @returns {Promise<any>} The API response data or throws an error
 */
export const handleApiResponse = async (apiCall, context = 'API call') => {
  try {
    const response = await apiCall;

    if (isSuccessfulResponse(response)) {
      return await response.json();
    } else {
      const error = await formatApiError(response);
      logError(error, context);
      throw error;
    }
  } catch (error) {
    // If it's already a formatted API error, log and rethrow
    if (error.status) {
      logError(error, context);
      throw error;
    }

    // Otherwise, it might be a network error or other issue
    logError(error, context);
    throw error;
  }
};