from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.middleware.error_handler import ErrorHandlerMiddleware
from src.config.settings import settings
import google.generativeai as genai
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
    allow_origins=["http://localhost:3000", "https://alishbaaslam.github.io/humanoid-robotics-textbook"],  # Allow frontend origin
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
    """Check if the configured Gemini model is available."""
    if settings.gemini_api_key or settings.openai_api_key:
        api_key = settings.gemini_api_key or settings.openai_api_key
        genai.configure(api_key=api_key)

        # Determine which model to check
        model_to_check = settings.GEMINI_MODEL if settings.GEMINI_MODEL else settings.agent_model

        try:
            # Test if the model is accessible
            test_model = genai.GenerativeModel(model_to_check)
            logger.info(f"Successfully verified model availability: {model_to_check}")
        except Exception as e:
            logger.error(f"Model {model_to_check} is not available: {e}")
            # Fail the startup if the model is unavailable
            raise RuntimeError(f"Gemini model {model_to_check} is not available: {e}")
    else:
        logger.warning("No Gemini or OpenAI API key provided - skipping model availability check")


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