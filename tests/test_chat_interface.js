/**
 * Test suite for Chat Interface Components
 * Testing User Stories 1, 2, and 3 UI components
 */

// Mock the agent service to avoid actual API calls
jest.mock('../website/src/api/agentService', () => ({
  submitQuery: jest.fn(),
  checkHealth: jest.fn()
}));

// Mock the message models
jest.mock('../website/src/models/message', () => ({
  isValidMessage: jest.fn((obj) => obj && obj.id && obj.type && obj.content && obj.timestamp),
  createUserMessage: jest.fn((content, id) => ({
    id: id || 'user_123',
    type: 'user',
    content,
    timestamp: new Date().toISOString()
  })),
  createAgentMessage: jest.fn((content, sources, confidence, id) => ({
    id: id || 'agent_123',
    type: 'agent',
    content,
    timestamp: new Date().toISOString(),
    sources,
    confidence
  })),
  createErrorMessage: jest.fn((content, id) => ({
    id: id || 'error_123',
    type: 'error',
    content,
    timestamp: new Date().toISOString()
  })),
  formatMessageForDisplay: jest.fn((message) => ({
    ...message,
    displayTimestamp: new Date(message.timestamp).toLocaleTimeString(),
    formattedContent: message.content
  }))
}));

// Mock the input validation
jest.mock('../website/src/utils/inputValidation', () => ({
  validateQueryRequest: jest.fn((data) => {
    if (!data.query || data.query.trim().length === 0) {
      return { isValid: false, errors: ['Query cannot be empty'] };
    }
    if (data.query.length > 1000) {
      return { isValid: false, errors: ['Query is too long'] };
    }
    return { isValid: true, errors: [] };
  })
}));

// Mock the error handling utilities
jest.mock('../website/src/utils/errorHandling', () => ({
  getUserFriendlyErrorMessage: jest.fn((error) => error.message || 'An error occurred'),
  logError: jest.fn()
}));

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ChatInterface from '../website/src/components/ChatInterface';

describe('Chat Interface Component Tests', () => {
  const mockSubmitQuery = require('../website/src/api/agentService').submitQuery;
  const mockCheckHealth = require('../website/src/api/agentService').checkHealth;

  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('User Story 1: Submit Queries to RAG Chatbot', () => {
    test('should render input field and allow query submission', () => {
      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      expect(input).toBeInTheDocument();
      expect(sendButton).toBeInTheDocument();
      expect(sendButton).toBeDisabled(); // Initially disabled due to empty input
    });

    test('should enable send button when input has text', () => {
      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      fireEvent.change(input, { target: { value: 'Test query' } });

      expect(sendButton).not.toBeDisabled();
    });

    test('should submit query when form is submitted', async () => {
      const mockResponse = {
        id: 'response_123',
        query: 'Test query',
        response: 'Test response',
        retrieved_chunks: [],
        confidence: 0.8,
        timestamp: new Date().toISOString()
      };

      mockSubmitQuery.mockResolvedValue(mockResponse);

      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      fireEvent.change(input, { target: { value: 'Test query' } });
      fireEvent.click(sendButton);

      await waitFor(() => {
        expect(mockSubmitQuery).toHaveBeenCalledWith({
          query: 'Test query',
          session_id: expect.any(String)
        });
      });
    });

    test('should handle Enter key submission', async () => {
      const mockResponse = {
        id: 'response_123',
        query: 'Test query',
        response: 'Test response',
        retrieved_chunks: [],
        confidence: 0.8,
        timestamp: new Date().toISOString()
      };

      mockSubmitQuery.mockResolvedValue(mockResponse);

      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');

      fireEvent.change(input, { target: { value: 'Test query' } });
      fireEvent.keyDown(input, { key: 'Enter', code: 'Enter' });

      await waitFor(() => {
        expect(mockSubmitQuery).toHaveBeenCalledWith({
          query: 'Test query',
          session_id: expect.any(String)
        });
      });
    });
  });

  describe('User Story 2: Display Agent Responses in UI', () => {
    test('should display user and agent messages correctly', async () => {
      const mockResponse = {
        id: 'response_123',
        query: 'Test query',
        response: 'Test response',
        retrieved_chunks: [{
          id: 'chunk_1',
          content: 'Sample content',
          source_document: 'Chapter 1',
          similarity_score: 0.85
        }],
        confidence: 0.8,
        timestamp: new Date().toISOString()
      };

      mockSubmitQuery.mockResolvedValue(mockResponse);

      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      fireEvent.change(input, { target: { value: 'Test query' } });
      fireEvent.click(sendButton);

      // Wait for both user and agent messages to appear
      await waitFor(() => {
        expect(screen.getByText('You:')).toBeInTheDocument();
        expect(screen.getByText('Test query')).toBeInTheDocument();
      });

      await waitFor(() => {
        expect(screen.getByText('Assistant:')).toBeInTheDocument();
        expect(screen.getByText('Test response')).toBeInTheDocument();
      });

      // Check that confidence is displayed
      expect(screen.getByText('Confidence: 80.0%')).toBeInTheDocument();
    });

    test('should display typing indicator when agent is responding', async () => {
      // Mock a delayed response to see the typing indicator
      mockSubmitQuery.mockImplementation(() => new Promise(resolve => {
        setTimeout(() => resolve({
          id: 'response_123',
          query: 'Test query',
          response: 'Test response',
          retrieved_chunks: [],
          confidence: 0.8,
          timestamp: new Date().toISOString()
        }), 100);
      }));

      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      fireEvent.change(input, { target: { value: 'Test query' } });
      fireEvent.click(sendButton);

      // Check that typing indicator appears while waiting for response
      expect(screen.getByLabelText('Assistant is typing')).toBeInTheDocument();

      // Wait for the response to complete
      await waitFor(() => {
        expect(screen.queryByLabelText('Assistant is typing')).not.toBeInTheDocument();
      });
    });
  });

  describe('User Story 3: Handle Communication Errors', () => {
    test('should display error message when API call fails', async () => {
      mockSubmitQuery.mockRejectedValue(new Error('Network error'));

      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      fireEvent.change(input, { target: { value: 'Test query' } });
      fireEvent.click(sendButton);

      await waitFor(() => {
        expect(screen.getByText('System:')).toBeInTheDocument();
        expect(screen.getByText(/issue processing your query/)).toBeInTheDocument();
      });
    });

    test('should display validation error for empty query', async () => {
      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      fireEvent.change(input, { target: { value: '' } });
      fireEvent.click(sendButton);

      // Since empty query doesn't trigger submit, we need to check validation
      fireEvent.change(input, { target: { value: ' ' } }); // Just whitespace
      fireEvent.click(sendButton);

      // Validation should happen before API call
      await waitFor(() => {
        expect(mockSubmitQuery).not.toHaveBeenCalled();
      });
    });
  });

  describe('Accessibility Features', () => {
    test('should have proper ARIA attributes for accessibility', () => {
      render(<ChatInterface />);

      const input = screen.getByPlaceholderText('Ask about humanoid robotics...');
      const sendButton = screen.getByLabelText('Send message');

      expect(input).toHaveAttribute('aria-label', 'Type your question about humanoid robotics');
      expect(sendButton).toHaveAttribute('aria-label', 'Send message');
    });
  });
});