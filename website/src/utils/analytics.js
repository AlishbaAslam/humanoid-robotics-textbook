/**
 * Analytics utilities for tracking user interactions with the chat interface
 */

/**
 * Track a user query submission
 * @param {string} query - The user's query text
 * @param {string} sessionId - The current session ID
 * @param {Object} metadata - Additional metadata about the query
 */
export const trackQuerySubmission = (query, sessionId, metadata = {}) => {
  try {
    // Log to console for development
    console.log('Analytics: Query submitted', {
      query: query.substring(0, 50) + (query.length > 50 ? '...' : ''),
      sessionId,
      timestamp: new Date().toISOString(),
      ...metadata
    });

    // In production, this would send data to an analytics service like:
    // - Google Analytics
    // - Mixpanel
    // - Custom analytics endpoint
    // Example:
    // analytics.track('query_submitted', {
    //   query: query,
    //   sessionId: sessionId,
    //   ...metadata
    // });
  } catch (error) {
    console.warn('Analytics tracking error:', error);
  }
};

/**
 * Track an agent response
 * @param {string} query - The original query
 * @param {string} response - The agent's response
 * @param {string} sessionId - The current session ID
 * @param {Object} metadata - Additional metadata about the response
 */
export const trackAgentResponse = (query, response, sessionId, metadata = {}) => {
  try {
    console.log('Analytics: Agent response received', {
      query: query.substring(0, 50) + (query.length > 50 ? '...' : ''),
      responseLength: response.length,
      sessionId,
      timestamp: new Date().toISOString(),
      ...metadata
    });

    // In production, send to analytics service
  } catch (error) {
    console.warn('Analytics tracking error:', error);
  }
};

/**
 * Track an error in the chat interface
 * @param {Error} error - The error object
 * @param {string} context - Context where the error occurred
 * @param {string} sessionId - The current session ID
 */
export const trackError = (error, context, sessionId) => {
  try {
    console.log('Analytics: Error occurred', {
      error: error.message || error.toString(),
      context,
      sessionId,
      timestamp: new Date().toISOString()
    });

    // In production, send error data to analytics/error tracking service
  } catch (trackError) {
    console.warn('Analytics tracking error:', trackError);
  }
};

/**
 * Track chat interface events
 * @param {string} eventName - Name of the event
 * @param {Object} properties - Event properties
 * @param {string} sessionId - The current session ID
 */
export const trackEvent = (eventName, properties = {}, sessionId) => {
  try {
    console.log('Analytics: Event tracked', {
      eventName,
      sessionId,
      timestamp: new Date().toISOString(),
      ...properties
    });

    // In production, send to analytics service
  } catch (error) {
    console.warn('Analytics tracking error:', error);
  }
};

/**
 * Initialize analytics tracking
 */
export const initAnalytics = () => {
  console.log('Analytics: Initialized');
  // In production, initialize the analytics service
  // e.g., analytics.identify() or similar initialization
};