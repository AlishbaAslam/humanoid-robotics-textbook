/**
 * Message Data Models for Frontend State Management
 * Defines the structure for messages in the chat interface
 */

/**
 * Represents a chat message
 * @typedef {Object} Message
 * @property {string} id - Unique identifier for the message
 * @property {'user'|'agent'|'error'} type - Type of the message
 * @property {string} content - The content of the message
 * @property {string} timestamp - ISO string timestamp of when the message was created
 * @property {Array<string>} [sources] - Optional array of source documents referenced
 * @property {number} [confidence] - Optional confidence level (0-1)
 */

/**
 * Represents a user message
 * @typedef {Object} UserMessage
 * @property {string} id - Unique identifier for the message
 * @property {'user'} type - Type of the message
 * @property {string} content - The content of the message
 * @property {string} timestamp - ISO string timestamp of when the message was created
 */

/**
 * Represents an agent message
 * @typedef {Object} AgentMessage
 * @property {string} id - Unique identifier for the message
 * @property {'agent'} type - Type of the message
 * @property {string} content - The content of the message
 * @property {string} timestamp - ISO string timestamp of when the message was created
 * @property {Array<string>} [sources] - Optional array of source documents referenced
 * @property {number} [confidence] - Optional confidence level (0-1)
 */

/**
 * Represents an error message
 * @typedef {Object} ErrorMessage
 * @property {string} id - Unique identifier for the message
 * @property {'error'} type - Type of the message
 * @property {string} content - The content of the message
 * @property {string} timestamp - ISO string timestamp of when the message was created
 */

/**
 * Validates a message object
 * @param {any} obj - Object to validate
 * @returns {boolean} True if the object is a valid message
 */
export const isValidMessage = (obj) => {
  if (!obj || typeof obj !== 'object') return false;

  // Check required properties
  if (!obj.id || typeof obj.id !== 'string') return false;
  if (!obj.type || !['user', 'agent', 'error'].includes(obj.type)) return false;
  if (!obj.content || typeof obj.content !== 'string') return false;
  if (!obj.timestamp || typeof obj.timestamp !== 'string') return false;

  // Validate timestamp format (ISO string)
  const date = new Date(obj.timestamp);
  if (isNaN(date.getTime())) return false;

  // Validate optional properties
  if (obj.sources !== undefined) {
    if (!Array.isArray(obj.sources) || !obj.sources.every(s => typeof s === 'string')) return false;
  }

  if (obj.confidence !== undefined) {
    if (typeof obj.confidence !== 'number' || obj.confidence < 0 || obj.confidence > 1) return false;
  }

  return true;
};

/**
 * Creates a user message
 * @param {string} content - The content of the message
 * @param {string} [id] - Optional ID for the message, will be generated if not provided
 * @returns {UserMessage} A valid user message object
 */
export const createUserMessage = (content, id = Date.now().toString()) => {
  return {
    id,
    type: 'user',
    content,
    timestamp: new Date().toISOString()
  };
};

/**
 * Creates an agent message
 * @param {string} content - The content of the message
 * @param {Array<string>} [sources] - Optional array of source documents
 * @param {number} [confidence] - Optional confidence level (0-1)
 * @param {string} [id] - Optional ID for the message, will be generated if not provided
 * @returns {AgentMessage} A valid agent message object
 */
export const createAgentMessage = (content, sources, confidence, id = Date.now().toString()) => {
  const message = {
    id,
    type: 'agent',
    content,
    timestamp: new Date().toISOString()
  };

  if (sources && Array.isArray(sources) && sources.length > 0) {
    message.sources = sources;
  }

  if (confidence !== undefined && typeof confidence === 'number' && confidence >= 0 && confidence <= 1) {
    message.confidence = confidence;
  }

  return message;
};

/**
 * Creates an error message
 * @param {string} content - The content of the message
 * @param {string} [id] - Optional ID for the message, will be generated if not provided
 * @returns {ErrorMessage} A valid error message object
 */
export const createErrorMessage = (content, id = Date.now().toString()) => {
  return {
    id,
    type: 'error',
    content,
    timestamp: new Date().toISOString()
  };
};

/**
 * Formats a message for display
 * @param {Message} message - The message to format
 * @returns {Object} Formatted message with additional display properties
 */
export const formatMessageForDisplay = (message) => {
  if (!isValidMessage(message)) {
    throw new Error('Invalid message object');
  }

  return {
    ...message,
    displayTimestamp: new Date(message.timestamp).toLocaleTimeString([], {
      hour: '2-digit',
      minute: '2-digit'
    }),
    formattedContent: message.content
  };
};