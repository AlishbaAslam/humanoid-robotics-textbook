import google.generativeai as genai
from typing import List, Dict, Any, Optional
from src.config.settings import settings
from src.models.query import RetrievedChunk
import logging

logger = logging.getLogger(__name__)


class OpenAIAgent:
    def __init__(self):
        # Configure the Google Generative AI SDK with the API key
        genai.configure(api_key=settings.gemini_api_key or settings.openai_api_key)
        # Determine which model to use: prioritize GEMINI_MODEL from env, fallback to agent_model
        model_to_use = settings.GEMINI_MODEL if settings.GEMINI_MODEL else settings.agent_model
        # Initialize the model with a fallback approach
        try:
            self.model = genai.GenerativeModel(model_to_use)
        except Exception as e:
            logger.warning(f"Could not initialize model {model_to_use}: {e}")
            # Try with a more standard model name format
            fallback_model = model_to_use
            if not fallback_model.startswith("models/"):
                fallback_model = f"models/{fallback_model}"
            logger.info(f"Trying fallback model name: {fallback_model}")
            self.model = genai.GenerativeModel(fallback_model)

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[RetrievedChunk],
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> str:
        """
        Generate a response using Google's Gemini API based on the query and retrieved chunks.
        """
        try:
            # Prepare the context from retrieved chunks
            context_text = ""
            if retrieved_chunks:
                context_text = "Relevant information from the humanoid robotics textbook:\n\n"
                for i, chunk in enumerate(retrieved_chunks, 1):
                    context_text += f"Source: {chunk.source_document}\n"
                    context_text += f"Content: {chunk.content}\n"
                    if chunk.page_number:
                        context_text += f"Page: {chunk.page_number}\n"
                    context_text += "---\n"
            else:
                context_text = "No relevant information found in the textbook for this query."

            # Prepare the full prompt
            prompt = f"""{context_text}

            Based on the above context, please answer the following question:
            {query}

            Please provide a comprehensive answer based only on the information provided in the context.
            If the information is not available in the context, please state that clearly.
            Always cite the source documents when providing information.
            """

            # Generate content using the model
            response = self.model.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.7,
                    max_output_tokens=1000,
                    top_p=1.0,
                )
            )

            # Extract and return the response
            return response.text

        except Exception as e:
            logger.error(f"Error generating response with Google Gemini: {e}")
            # Return a helpful fallback response instead of crashing
            fallback_response = (
                f"I'm sorry, but I encountered an issue processing your query: '{query}'. "
                "This may be due to API configuration issues. "
                "If relevant information was found in the textbook, it would have been displayed here. "
                "Please contact the system administrator to check the AI model configuration."
            )

            if retrieved_chunks:
                fallback_response += f"\n\nFound {len(retrieved_chunks)} relevant document chunks that could have been used for context."

            return fallback_response


# Global instance
openai_agent = OpenAIAgent()