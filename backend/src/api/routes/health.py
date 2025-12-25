from fastapi import APIRouter
from typing import Dict
from datetime import datetime
import asyncio
from src.config.settings import settings

router = APIRouter()

@router.get("/health", summary="Health check endpoint")
async def health_check() -> Dict:
    """
    Check the health status of the agent service
    """
    # Check the status of various components
    api_key_status = "configured" if settings.openrouter_api_key else "not configured"
    model_status = settings.agent_model if settings.agent_model else "not configured"

    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "details": {
            "api_status": "operational",
            "version": "1.0.0",
            "openrouter_api_key": api_key_status,
            "configured_model": model_status,
            "openrouter_base_url": getattr(settings, 'openrouter_base_url', 'not configured')
        }
    }

@router.get("/ready", summary="Readiness check endpoint")
async def readiness_check() -> Dict:
    """
    Check if the service is ready to handle requests
    """
    # Check if the service has the necessary configuration to operate
    has_api_key = bool(settings.openrouter_api_key)
    has_model = bool(settings.agent_model)

    # For readiness, we might consider the service ready if basic config is present
    readiness_status = "ready" if (has_api_key or settings.agent_model) else "not ready"

    return {
        "status": readiness_status,
        "timestamp": datetime.now().isoformat(),
        "details": {
            "openrouter_api_key_configured": has_api_key,
            "agent_model_configured": has_model,
            "requires_api_key_for_full_functionality": True
        }
    }