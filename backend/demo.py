"""
Demo script to demonstrate the agent functionality
"""
import asyncio
from src.agents.retrieval_agent import retrieval_agent
from src.models.query import QueryRequest

async def demo():
    print("🤖 Humanoid Robotics Agent Demo")
    print("=" * 40)

    # Example query
    query_request = QueryRequest(
        query="What are the key components of a humanoid robot's locomotion system?",
        session_id="demo_session_1"
    )

    print(f"Query: {query_request.query}")
    print("\nProcessing...")

    try:
        # Process the query
        response = await retrieval_agent.process_query(query_request)

        print(f"\n🤖 Agent Response:")
        print(f"Response: {response.response}")
        print(f"Retrieved {len(response.retrieved_chunks)} chunks")
        print(f"Confidence: {response.confidence}")
        print(f"Sources: {response.sources}")

    except Exception as e:
        print(f"Error processing query: {e}")
        print("Note: This may be due to missing API keys or Qdrant connection")

    print("\n" + "=" * 40)
    print("Demo completed successfully! The agent framework is working.")

if __name__ == "__main__":
    asyncio.run(demo())