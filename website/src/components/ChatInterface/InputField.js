import React from 'react';
import './chat-interface.css';

/**
 * InputField Component
 * A reusable input field component for the chat interface
 */
const InputField = ({
  value,
  onChange,
  onSubmit,
  disabled = false,
  placeholder = "Type your message...",
  ariaLabel = "Type your message"
}) => {
  const handleSubmit = (e) => {
    e.preventDefault();
    if (value.trim() && !disabled) {
      onSubmit(e);
    }
  };

  const handleKeyDown = (e) => {
    // Submit on Enter key (but allow Shift+Enter for new lines)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (value.trim() && !disabled) {
        onSubmit(e);
      }
    }
  };

  return (
    <form onSubmit={handleSubmit} className="chat-input-form">
      <input
        type="text"
        value={value}
        onChange={(e) => onChange(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder={placeholder}
        disabled={disabled}
        aria-label={ariaLabel}
        className="chat-input"
      />
      <button
        type="submit"
        disabled={!value.trim() || disabled}
        className="chat-send-button"
        aria-label="Send message"
      >
        {disabled ? 'Sending...' : 'Send'}
      </button>
    </form>
  );
};

export default InputField;