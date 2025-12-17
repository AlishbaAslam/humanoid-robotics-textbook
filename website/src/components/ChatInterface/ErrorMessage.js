import React from 'react';
import './chat-interface.css';

/**
 * ErrorMessage Component
 * Displays error messages in the chat interface
 */
const ErrorMessage = ({ message, details = null, timestamp = null }) => {
  // Format the error message for display
  const formatErrorMessage = (msg) => {
    if (!msg) return 'An unknown error occurred. Please try again.';

    // Clean up error message to remove technical details
    let cleanMsg = msg;

    // Remove technical error prefixes
    cleanMsg = cleanMsg.replace(/^Error:\s*/, '');
    cleanMsg = cleanMsg.replace(/^Failed to submit query:\s*/, '');
    cleanMsg = cleanMsg.replace(/^API request failed:\s*/, '');

    return cleanMsg;
  };

  // Format timestamp for display
  const formatTimestamp = (ts) => {
    if (!ts) return null;

    try {
      const date = new Date(ts);
      return date.toLocaleTimeString([], {
        hour: '2-digit',
        minute: '2-digit'
      });
    } catch (e) {
      console.warn('Invalid timestamp provided to ErrorMessage:', ts);
      return null;
    }
  };

  return (
    <div className="message message-error" role="alert" aria-live="assertive">
      <div className="message-content">
        <span className="message-author">System:</span>
        <div className="message-text error-text">{formatErrorMessage(message)}</div>

        {details && (
          <details className="error-details">
            <summary>Error details</summary>
            <pre className="error-details-content">{details}</pre>
          </details>
        )}

        {timestamp && (
          <div className="message-timestamp">
            {formatTimestamp(timestamp)}
          </div>
        )}
      </div>
    </div>
  );
};

export default ErrorMessage;