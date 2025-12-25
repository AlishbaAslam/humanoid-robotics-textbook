/**
 * Test suite for Agent Service API communication
 * Testing User Stories 1, 2, and 3 frontend components
 */

// Mock the fetch API for testing
global.fetch = jest.fn();

describe('Agent Service API Tests', () => {
  let agentService;

  beforeEach(async () => {
    // Dynamically import to allow fetch mocking
    agentService = await import('../website/src/api/agentService');
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('User Story 1: Submit Queries to RAG Chatbot', () => {
    test('should successfully submit a query and receive a response', async () => {
      const mockResponse = {
        id: 'response_123',
        query: 'What are the key components of a humanoid robot?',
        response: 'The key components include actuators, sensors, and control systems.',
        retrieved_chunks: [],
        confidence: 0.8,
        timestamp: '2025-12-24T10:00:00Z'
      };

      global.fetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

      const queryData = {
        query: 'What are the key components of a humanoid robot?',
        session_id: 'test_session_123'
      };

      const result = await agentService.submitQuery(queryData);

      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('/api/agent/query'),
        expect.objectContaining({
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(queryData)
        })
      );
      expect(result).toEqual(mockResponse);
    });

    test('should handle network errors gracefully', async () => {
      global.fetch.mockRejectedValue(new Error('Network error'));

      const queryData = {
        query: 'Test query',
        session_id: 'test_session_123'
      };

      await expect(agentService.submitQuery(queryData)).rejects.toThrow('Failed to submit query');
    });
  });

  describe('User Story 2: Display Agent Responses in UI', () => {
    test('should format responses correctly for UI display', async () => {
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
        timestamp: '2025-12-24T10:00:00Z'
      };

      global.fetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockResponse)
      });

      const result = await agentService.submitQuery({ query: 'Test', session_id: 'test' });

      // Verify that the response contains all required fields for UI display
      expect(result).toHaveProperty('id');
      expect(result).toHaveProperty('query');
      expect(result).toHaveProperty('response');
      expect(result).toHaveProperty('retrieved_chunks');
      expect(result).toHaveProperty('confidence');
      expect(result).toHaveProperty('sources');
      expect(result).toHaveProperty('timestamp');
    });
  });

  describe('User Story 3: Handle Communication Errors', () => {
    test('should handle API errors with proper error formatting', async () => {
      global.fetch.mockResolvedValue({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'
      });

      const queryData = {
        query: 'Test query',
        session_id: 'test_session_123'
      };

      await expect(agentService.submitQuery(queryData)).rejects.toThrow('API request failed');
    });

    test('should handle 400 errors appropriately', async () => {
      global.fetch.mockResolvedValue({
        ok: false,
        status: 400,
        statusText: 'Bad Request'
      });

      await expect(agentService.submitQuery({ query: '', session_id: 'test' })).rejects.toThrow('API request failed');
    });
  });

  describe('Health Check', () => {
    test('should return health status when calling checkHealth', async () => {
      const mockHealthResponse = {
        status: 'healthy',
        timestamp: '2025-12-24T10:00:00Z'
      };

      global.fetch.mockResolvedValue({
        ok: true,
        json: () => Promise.resolve(mockHealthResponse)
      });

      const result = await agentService.checkHealth();

      expect(global.fetch).toHaveBeenCalledWith(
        expect.stringContaining('/health'),
        expect.objectContaining({
          method: 'GET',
          headers: { 'Content-Type': 'application/json' }
        })
      );
      expect(result).toEqual(mockHealthResponse);
    });
  });
});