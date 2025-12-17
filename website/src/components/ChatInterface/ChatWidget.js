import React, { useState } from 'react';
import ChatInterface from './index';
import ChatToggleButton from './ChatToggleButton';
import './chat-widget.css';

/**
 * ChatWidget Component
 * Floating chat widget that can be toggled open/closed
 */
const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  return (
    <>
      <div className={`chat-widget ${isOpen ? 'chat-widget--open' : 'chat-widget--closed'}`}>
        <div className="chat-widget-header">
          <h3>Humanoid Robotics Assistant</h3>
          <button
            className="chat-widget-close-button"
            onClick={closeChat}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>
        <div className="chat-widget-content">
          <ChatInterface />
        </div>
      </div>
      <ChatToggleButton onClick={toggleChat} isOpen={isOpen} />
    </>
  );
};

export default ChatWidget;