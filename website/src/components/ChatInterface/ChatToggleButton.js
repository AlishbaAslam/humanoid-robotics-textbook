import React from 'react';
import './chat-widget.css';

/**
 * ChatToggleButton Component
 * Floating button that toggles the chat widget visibility
 */
const ChatToggleButton = ({ onClick, isOpen }) => {
  return (
    <button
      className={`chat-toggle-button ${isOpen ? 'chat-toggle-button--open' : ''}`}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      aria-expanded={isOpen}
      role="button"
    >
      <div className="chat-icon">
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      </div>
    </button>
  );
};

export default ChatToggleButton;