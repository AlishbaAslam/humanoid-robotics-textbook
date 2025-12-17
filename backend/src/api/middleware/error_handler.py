from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Callable
import logging

logger = logging.getLogger(__name__)


class ErrorHandlerMiddleware:
    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            return await self.app(scope, receive, send)

        request = Request(scope)
        try:
            response = await self.app(scope, receive, send)
            return response
        except HTTPException as e:
            logger.error(f"HTTP Exception: {e.status_code} - {e.detail}")
            return JSONResponse(
                status_code=e.status_code,
                content={
                    "error": "HTTP_ERROR",
                    "message": str(e.detail),
                    "status_code": e.status_code
                }
            )
        except Exception as e:
            logger.error(f"Unhandled Exception: {str(e)}", exc_info=True)
            return JSONResponse(
                status_code=500,
                content={
                    "error": "INTERNAL_SERVER_ERROR",
                    "message": "An unexpected error occurred",
                    "status_code": 500
                }
            )


async def handle_validation_error(request: Request, exc: HTTPException):
    """Handle validation errors specifically"""
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": "VALIDATION_ERROR",
            "message": str(exc.detail),
            "status_code": exc.status_code
        }
    )