from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.middleware.error_handler import ErrorHandlerMiddleware
from src.config.settings import settings
import logging

logger = logging.getLogger(__name__)

# Import routers
from src.api.routes import health, agent

# Create FastAPI app
app = FastAPI(
    title="Humanoid Robotics Agent API",
    description="API for interacting with the humanoid robotics content agent",
    version="1.0.0",
    openapi_url="/openapi.json",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Add middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "https://alishbaaslam.github.io",
        "https://alishbaaslam.github.io/humanoid-robotics-textbook"
    ],  # Allow frontend origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add error handler middleware
app.add_middleware(ErrorHandlerMiddleware)

# Include routers
app.include_router(health.router, tags=["health"])
app.include_router(agent.router, prefix="/api/agent", tags=["agent"])

@app.on_event("startup")
async def startup_event():
    """Verify OpenRouter API key is configured."""

    if settings.openrouter_api_key:
        logger.info("OpenRouter API key is configured")
    else:
        logger.warning("No OPENROUTER_API_KEY provided - OpenRouter functionality will be limited")


@app.get("/")
async def root():
    return {"message": "Welcome to the Humanoid Robotics Agent API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.api.main:app",
        host=settings.host,
        port=settings.port,
        reload=True
    )