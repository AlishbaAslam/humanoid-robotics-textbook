/**
 * Frontend Input Validation Utilities
 * Contains utilities for validating user input before sending to backend
 */

/**
 * Validates a query text input
 * @param {string} queryText - The query text to validate
 * @returns {Object} Validation result with isValid boolean and optional error message
 */
export const validateQueryText = (queryText) => {
  // Check if query is provided
  if (!queryText) {
    return {
      isValid: false,
      error: 'Query is required'
    };
  }

  // Convert to string if not already
  const text = String(queryText);

  // Check if query is empty after trimming
  if (!text.trim()) {
    return {
      isValid: false,
      error: 'Query cannot be empty or contain only whitespace'
    };
  }

  // Check minimum length
  if (text.length < 1) {
    return {
      isValid: false,
      error: 'Query must be at least 1 character long'
    };
  }

  // Check maximum length (matching backend validation)
  if (text.length > 1000) {
    return {
      isValid: false,
      error: 'Query must be no more than 1000 characters long'
    };
  }

  // Check for potentially dangerous content
  if (containsPotentialInjection(text)) {
    return {
      isValid: false,
      error: 'Query contains potentially harmful content'
    };
  }

  return {
    isValid: true,
    error: null
  };
};

/**
 * Checks if text contains potential injection patterns
 * @param {string} text - Text to check for injection patterns
 * @returns {boolean} True if potential injection patterns are found
 */
export const containsPotentialInjection = (text) => {
  const injectionPatterns = [
    /<script/i,  // XSS attempts
    /javascript:/i,  // JavaScript URLs
    /vbscript:/i,  // VBScript URLs
    /on\w+\s*=/i,  // Event handlers
    /<iframe/i,  // Frame injection
    /<object/i,  // Object injection
    /<embed/i,  // Embed injection
    /<svg/i,  // SVG injection
    /<link/i,  // Link injection
    /<meta/i,  // Meta injection
    /<!--/i,  // Comment injection
  ];

  const lowerText = text.toLowerCase();
  for (const pattern of injectionPatterns) {
    if (pattern.test(lowerText)) {
      return true;
    }
  }

  return false;
};

/**
 * Sanitizes query text by removing potentially harmful content
 * @param {string} queryText - The query text to sanitize
 * @returns {string} Sanitized query text
 */
export const sanitizeQueryText = (queryText) => {
  if (!queryText) return '';

  let sanitized = String(queryText);

  // Remove script tags and their content
  sanitized = sanitized.replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '');

  // Remove event handlers
  sanitized = sanitized.replace(/on\w+\s*=\s*["'][^"']*["']/gi, '');

  // Remove javascript: and vbscript: URLs
  sanitized = sanitized.replace(/(javascript:|vbscript:)/gi, '');

  // Remove iframe, object, embed, svg tags
  sanitized = sanitized.replace(/<(iframe|object|embed|svg)\b[^>]*>.*?<\/\1>/gi, '');
  sanitized = sanitized.replace(/<(iframe|object|embed|svg)\b[^>]*\/?>/gi, '');

  // Trim whitespace
  return sanitized.trim();
};

/**
 * Validates a session ID
 * @param {string} sessionId - The session ID to validate
 * @returns {Object} Validation result with isValid boolean and optional error message
 */
export const validateSessionId = (sessionId) => {
  if (!sessionId) {
    return {
      isValid: true, // Session ID is optional
      error: null
    };
  }

  const id = String(sessionId);

  // Check length
  if (id.length < 8 || id.length > 64) {
    return {
      isValid: false,
      error: 'Session ID must be between 8 and 64 characters long'
    };
  }

  // Check for valid characters (alphanumeric, hyphens, underscores)
  if (!/^[a-zA-Z0-9_-]+$/.test(id)) {
    return {
      isValid: false,
      error: 'Session ID contains invalid characters. Only alphanumeric, hyphens, and underscores are allowed.'
    };
  }

  return {
    isValid: true,
    error: null
  };
};

/**
 * Validates a user ID
 * @param {string} userId - The user ID to validate
 * @returns {Object} Validation result with isValid boolean and optional error message
 */
export const validateUserId = (userId) => {
  if (!userId) {
    return {
      isValid: true, // User ID is optional
      error: null
    };
  }

  const id = String(userId);

  // Check length
  if (id.length < 4 || id.length > 32) {
    return {
      isValid: false,
      error: 'User ID must be between 4 and 32 characters long'
    };
  }

  // Check for valid characters (alphanumeric, hyphens, underscores)
  if (!/^[a-zA-Z0-9_-]+$/.test(id)) {
    return {
      isValid: false,
      error: 'User ID contains invalid characters. Only alphanumeric, hyphens, and underscores are allowed.'
    };
  }

  return {
    isValid: true,
    error: null
  };
};

/**
 * Validates a full query request object
 * @param {Object} queryRequest - The query request object to validate
 * @param {string} queryRequest.query - The query text
 * @param {string} [queryRequest.session_id] - Optional session ID
 * @param {string} [queryRequest.user_id] - Optional user ID
 * @returns {Object} Validation result with isValid boolean and array of error messages
 */
export const validateQueryRequest = (queryRequest) => {
  const errors = [];

  // Validate query text
  const queryValidation = validateQueryText(queryRequest.query);
  if (!queryValidation.isValid) {
    errors.push(queryValidation.error);
  }

  // Validate session ID if provided
  if (queryRequest.session_id) {
    const sessionValidation = validateSessionId(queryRequest.session_id);
    if (!sessionValidation.isValid) {
      errors.push(sessionValidation.error);
    }
  }

  // Validate user ID if provided
  if (queryRequest.user_id) {
    const userValidation = validateUserId(queryRequest.user_id);
    if (!userValidation.isValid) {
      errors.push(userValidation.error);
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
};

/**
 * Truncates text to a specified maximum length
 * @param {string} text - Text to truncate
 * @param {number} maxLength - Maximum length
 * @param {string} suffix - Suffix to add if truncated (default: '...')
 * @returns {string} Truncated text
 */
export const truncateText = (text, maxLength, suffix = '...') => {
  if (!text || text.length <= maxLength) {
    return text;
  }

  return text.substring(0, maxLength - suffix.length) + suffix;
};

/**
 * Formats query text for display by truncating if necessary
 * @param {string} queryText - The query text to format
 * @returns {string} Formatted query text
 */
export const formatQueryTextForDisplay = (queryText) => {
  if (!queryText) return '';

  // Sanitize and truncate for display
  const sanitized = sanitizeQueryText(queryText);
  return truncateText(sanitized, 100);
};