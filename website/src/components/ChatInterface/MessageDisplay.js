import React from 'react';
import './chat-interface.css';

/**
 * MessageDisplay Component
 * Displays a single chat message with appropriate styling based on type
 */
const MessageDisplay = ({ message }) => {
  if (!message || !message.id || !message.type || !message.content) {
    console.warn('Invalid message object provided to MessageDisplay');
    return null;
  }

  const formatTimestamp = (timestamp) => {
    if (!timestamp) return '';
    try {
      const date = new Date(timestamp);
      return date.toLocaleTimeString([], {
        hour: '2-digit',
        minute: '2-digit'
      });
    } catch (e) {
      console.warn('Invalid timestamp provided to MessageDisplay:', timestamp);
      return '';
    }
  };

  return (
    <div
      key={message.id}
      className={`message message-${message.type}`}
      aria-live={message.type === 'agent' ? 'polite' : 'off'}
      role="status"
    >
      <div className="message-content">
        {message.type === 'user' && <span className="message-author">You:</span>}
        {message.type === 'agent' && <span className="message-author">Assistant:</span>}
        <div className="message-text">{message.content}</div>

        {message.sources && message.sources.length > 0 && (
          <div className="message-sources">
            <strong>Sources:</strong> {message.sources.join(', ')}
          </div>
        )}

        {message.confidence !== undefined && (
          <div className="message-confidence">
            Confidence: {(message.confidence * 100).toFixed(1)}%
          </div>
        )}

        {/* Display timestamp if available */}
        {message.timestamp && (
          <div className="message-timestamp">
            {formatTimestamp(message.timestamp)}
          </div>
        )}
      </div>
    </div>
  );
};

export default MessageDisplay;