import openai
from typing import List, Dict, Any, Optional
from src.config.settings import settings
from src.models.query import RetrievedChunk
import logging
import uuid
from datetime import datetime, timezone

logger = logging.getLogger(__name__)


class OpenRouterAgent:
    """OpenAI Agent SDK pattern implementation using OpenRouter API."""

    def __init__(self):
        # Configure the OpenRouter client using OpenAI SDK
        if not settings.openrouter_api_key:
            logger.warning("No OPENROUTER_API_KEY provided - OpenRouter client will not be initialized")
            self.client = None
        else:
            try:
                self.client = openai.AsyncOpenAI(
                    api_key=settings.openrouter_api_key,
                    base_url="https://openrouter.ai/api/v1"
                )
                logger.info("Initialized OpenRouter client with AsyncOpenAI")
            except Exception as e:
                logger.error(f"Failed to initialize OpenRouter client: {e}")
                self.client = None

        # Determine primary model to use
        self.model = getattr(settings, 'OPENROUTER_MODEL', 'tngtech/deepseek-r1t2-chimera:free') or 'tngtech/deepseek-r1t2-chimera:free'

        # Initialize the assistant if OpenRouter client is available
        self.assistant = None
        if self.client:
            try:
                # Create an assistant using the OpenAI Assistants API
                self.assistant = self.client.beta.assistants.create(
                    name="Humanoid Robotics Assistant",
                    description="An AI assistant specialized in humanoid robotics content from the textbook",
                    model=self.model,
                    instructions="""You are an expert assistant for humanoid robotics content.
                    You will be provided with relevant information from a humanoid robotics textbook.
                    Use this information to answer user questions accurately and comprehensively.
                    ALWAYS cite your sources using this format: (Source: [module name], Chapter: [Chapter name])
                    At the end of your response, include a summary of sources cited in this format: Sources cited: [list of document names]
                    If the information is not available in the provided context, clearly state that.
                    Structure your responses with direct answers, supporting details, and proper citations."""
                )
                logger.info(f"Created assistant with ID: {self.assistant.id}")
            except Exception as e:
                logger.error(f"Failed to create assistant via OpenRouter: {e}")
                # Fall back to using chat completions directly
                self.assistant = None

    async def generate_response(
        self,
        query: str,
        retrieved_chunks: List[RetrievedChunk],
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> str:
        """
        Generate a response using OpenAI Assistant API via OpenRouter based on the query and retrieved chunks.
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

            Provide a simple, concise answer in ONE paragraph. Do not use numbered lists, headings, or sections like "Direct answer" or "Supporting details".
            If relevant information is available in the context, include it naturally in your response.
            ALWAYS cite your sources using this format: (Source: [full module name], Chapter: [full chapter name])
            """

            # Use OpenAI Assistant API via OpenRouter if available
            if self.client and self.assistant:
                try:
                    # Create a thread for the conversation
                    thread = await self.client.beta.threads.create(
                        messages=[
                            {
                                "role": "user",
                                "content": prompt,
                            }
                        ]
                    )

                    # Run the assistant on the thread
                    run = await self.client.beta.threads.runs.create_and_poll(
                        thread_id=thread.id,
                        assistant_id=self.assistant.id,
                    )

                    # Wait for the run to complete and get the messages
                    messages = await self.client.beta.threads.messages.list(
                        thread_id=thread.id
                    )

                    # Get the latest assistant message
                    response_content = ""
                    for msg in messages.data:
                        if msg.role == "assistant":
                            # Extract text content from the message
                            for content_block in msg.content:
                                if content_block.type == "text":
                                    response_content = content_block.text.value
                                    break
                            break

                    if response_content and len(response_content) >= 10:
                        return response_content
                    else:
                        logger.warning("OpenRouter Assistant response was too short")
                except Exception as assistant_error:
                    logger.error(f"Error with OpenAI Assistant API via OpenRouter: {assistant_error}")
                    # Fall back to chat completions if assistant API fails

            # Fall back to chat completions if assistant is not available or fails
            if self.client:
                try:
                    # Prepare messages for OpenRouter
                    messages = []

                    # Add conversation history if available
                    if conversation_history:
                        for msg in conversation_history:
                            role = "user" if msg["role"] == "user" else "assistant"
                            messages.append({"role": role, "content": msg["content"]})

                    # Add the current prompt
                    messages.append({"role": "user", "content": prompt})

                    # Generate response using OpenRouter
                    response = await self.client.chat.completions.create(
                        model=self.model,
                        messages=messages,
                        temperature=0.7,
                        max_tokens=2000,
                        top_p=1.0,
                    )

                    response_content = response.choices[0].message.content

                    if response_content and len(response_content) >= 10:
                        return response_content
                    else:
                        logger.warning("OpenRouter chat completion response was too short")
                except Exception as openrouter_error:
                    logger.error(f"Error with OpenRouter API: {openrouter_error}")

            # If OpenRouter service fails, return a helpful response with the retrieved chunks
            logger.warning("OpenRouter failed, returning response based on retrieved chunks")
            if retrieved_chunks:
                # Return a simple response using the first chunk of content
                first_chunk_content = retrieved_chunks[0].content[:500]  # Limit length
                return f"Based on the textbook: {first_chunk_content}"
            else:
                return f"Sorry, I couldn't find information about '{query}' in the textbook."

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            # Return a simple fallback response
            if retrieved_chunks:
                # Provide a simple response using the first chunk of content
                first_chunk_content = retrieved_chunks[0].content[:500]  # Limit length
                fallback_response = f"Based on the textbook: {first_chunk_content}"
            else:
                fallback_response = f"Sorry, I couldn't find information about '{query[:50]}...' in the textbook."

            return fallback_response


class Agent:
    """Agent class following OpenAI Agents SDK pattern."""

    def __init__(self, assistant_config=None):
        self.agent = OpenRouterAgent()

    async def run(self, query: str, retrieved_chunks: List[RetrievedChunk], conversation_history: Optional[List[Dict[str, str]]] = None):
        """Run the agent with the given query and context."""
        return await self.agent.generate_response(query, retrieved_chunks, conversation_history)


class Runner:
    """Runner class following OpenAI Agents SDK pattern."""

    def __init__(self, agent: Agent):
        self.agent = agent

    async def run(self, query: str, retrieved_chunks: List[RetrievedChunk], conversation_history: Optional[List[Dict[str, str]]] = None):
        """Run the agent with the given query and context."""
        return await self.agent.run(query, retrieved_chunks, conversation_history)

    async def run_sync(self, query: str, retrieved_chunks: List[RetrievedChunk], conversation_history: Optional[List[Dict[str, str]]] = None):
        """Synchronous run method for compatibility."""
        return await self.agent.run(query, retrieved_chunks, conversation_history)


class RunConfig:
    """RunConfig class following OpenAI Agents SDK pattern."""

    def __init__(self, model: str = None, temperature: float = 0.7, max_tokens: int = 2000):
        self.model = model or getattr(settings, 'OPENROUTER_MODEL', 'tngtech/deepseek-r1t2-chimera:free')
        self.temperature = temperature
        self.max_tokens = max_tokens


# Global instance using the new pattern
openrouter_agent = OpenRouterAgent()

# Also create the Agent/Runner/RunConfig pattern for compatibility
agent = Agent()
runner = Runner(agent)
run_config = RunConfig()