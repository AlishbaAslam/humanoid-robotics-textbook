"""
Basic tests for the agent API to verify functionality.
"""
import pytest
from fastapi.testclient import TestClient
from src.api.main import app

client = TestClient(app)

def test_health_endpoint():
    """Test the health endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert "timestamp" in data


def test_root_endpoint():
    """Test the root endpoint"""
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert "message" in data
    assert "Welcome" in data["message"]


def test_agent_query_endpoint_exists():
    """Test that the agent query endpoint exists"""
    # This should return 422 (validation error) because we didn't send proper data,
    # but it should not return 404 (not found)
    response = client.post("/api/agent/query", json={})
    assert response.status_code == 422  # Validation error, not 404


if __name__ == "__main__":
    pytest.main([__file__])