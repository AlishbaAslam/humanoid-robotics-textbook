import React, { useState, useRef, useEffect } from 'react';
import { isValidMessage, createUserMessage, createAgentMessage, createErrorMessage, formatMessageForDisplay } from '../../models/message';
import { validateQueryRequest } from '../../utils/inputValidation';
import { getSessionState, clearSession } from '../../api/agentService';
import { getUserFriendlyErrorMessage, logError } from '../../utils/errorHandling';
import { trackQuerySubmission, trackAgentResponse, trackError, initAnalytics } from '../../utils/analytics';
import { initAccessibility } from '../../utils/accessibility';
import './chat-interface.css';

/**
 * ChatInterface Component
 * Main component for the RAG chatbot interface
 */
const ChatInterface = ({ initialMessages = [], initialSessionId = null }) => {
  const [messages, setMessages] = useState(initialMessages);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [isSending, setIsSending] = useState(false); // More specific loading state
  const [sessionId, setSessionId] = useState(initialSessionId || null);

useEffect(() => {
  // Initialize analytics and accessibility features
  initAnalytics();
  initAccessibility();

  // Browser me hi run kare
  if (!sessionId) {
    const storedSessionId = sessionStorage.getItem('session_id') || `session_${Date.now()}`;
    setSessionId(storedSessionId);
    sessionStorage.setItem('session_id', storedSessionId);
  }
}, [sessionId]);
  const messagesEndRef = useRef(null);

  // Store session ID in sessionStorage when it changes
  useEffect(() => {
    sessionStorage.setItem('session_id', sessionId);
  }, [sessionId]);

  // Scroll to bottom of messages when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isSending) return;

    // Validate input before sending
    const validation = validateQueryRequest({ query: inputValue });
    if (!validation.isValid) {
      const errorMessage = createErrorMessage(
        `Invalid query: ${validation.errors.join(', ')}`,
        `error_${Date.now()}`
      );
      setMessages(prev => [...prev, errorMessage]);
      trackError(new Error(`Invalid query: ${validation.errors.join(', ')}`), 'Input validation', sessionId);
      return;
    }

    // Track the query submission
    trackQuerySubmission(inputValue, sessionId);

    // Create and add user message to chat
    const userMessage = createUserMessage(inputValue, `user_${Date.now()}`);
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsSending(true);
    setError(null);

    try {
      // Import the API service function
      const { submitQuery } = await import('../../api/agentService');

      // Send query to backend with current session ID
      const response = await submitQuery({
        query: inputValue,
        session_id: sessionId
      });

      // Create and add agent response to chat with defensive checks for potential null/undefined values
      const agentMessage = createAgentMessage(
        response.response || 'No response generated',
        Array.isArray(response.sources) ? response.sources : (response.sources ? [response.sources] : []),
        (response.confidence !== null && response.confidence !== undefined) ? response.confidence : null,
        response.id || `agent_${Date.now()}`
      );

      setMessages(prev => [...prev, agentMessage]);

      // Track the agent response with defensive checks
      trackAgentResponse(inputValue, response.response, sessionId, {
        confidence: response.confidence !== undefined ? response.confidence : null,
        sources: Array.isArray(response.sources) ? response.sources : (response.sources ? [response.sources] : [])
      });

      // Update session ID if returned from backend
      if (response.session_id) {
        setSessionId(response.session_id);
      }
    } catch (err) {
      console.error('Error submitting query:', err);

      // Track the error
      trackError(err, 'Query submission', sessionId);

      // Use the error handling utilities to get a user-friendly message
      const userFriendlyMessage = getUserFriendlyErrorMessage(err);
      setError(userFriendlyMessage);

      // Log the error for debugging
      logError(err, 'ChatInterface submitQuery');

      // Add error message to chat
      const errorMessage = createErrorMessage(
        `Sorry, there was an issue processing your query: ${userFriendlyMessage}`,
        `error_${Date.now()}`
      );
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsSending(false);
    }
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>Humanoid Robotics Assistant</h3>
      </div>

      <div className="chat-messages">
        {messages.map((message) => (
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

              {message.confidence !== undefined && message.confidence !== null && typeof message.confidence === 'number' && (
                <div className="message-confidence">
                  Confidence: {(message.confidence * 100).toFixed(1)}%
                </div>
              )}

              {/* Display timestamp if available */}
              {message.timestamp && (
                <div className="message-timestamp">
                  {new Date(message.timestamp).toLocaleTimeString([], {
                    hour: '2-digit',
                    minute: '2-digit'
                  })}
                </div>
              )}
            </div>
          </div>
        ))}

        {isSending && (
          <div className="message message-agent" aria-label="Assistant is typing">
            <div className="message-content">
              <span className="message-author">Assistant:</span>
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}

        {error && (
          <div className="message message-error">
            <div className="message-content">
              <span className="message-author">System:</span>
              <div className="message-text error-text">{error}</div>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className="chat-input-form">
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask about humanoid robotics..."
          disabled={isSending}
          aria-label="Type your question about humanoid robotics"
          className="chat-input"
        />
        <button
          type="submit"
          disabled={!inputValue.trim() || isSending}
          className="chat-send-button"
          aria-label="Send message"
        >
          {isSending ? 'Sending...' : 'Send'}
        </button>
      </form>
    </div>
  );
};

export default ChatInterface;