#!/usr/bin/env python3
"""
Script to create the Qdrant collection for humanoid robotics content.
This ensures the collection exists before attempting retrieval operations.
"""

import asyncio
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from src.config.settings import settings
from src.services.qdrant_service import QdrantService


async def create_collection():
    """Create the Qdrant collection if it doesn't exist."""
    print(f"Creating Qdrant collection: {settings.qdrant_collection_name}")
    print(f"Qdrant URL: {settings.qdrant_url}")

    if not settings.qdrant_url:
        print("Error: QDRANT_URL not set in environment variables")
        return False

    qdrant_service = QdrantService()

    try:
        # Create the collection
        success = await qdrant_service.create_collection()

        if success:
            print(f"✓ Collection '{settings.qdrant_collection_name}' is ready!")

            # Check how many points are in the collection
            count = await qdrant_service.count_points()
            print(f"✓ Collection contains {count} points")

            if count == 0:
                print("! Warning: Collection is empty. You need to run the ingestion process to add content.")
                print("  Run: python main.py --url <your_website_url>")

            return True
        else:
            print(f"✗ Failed to create collection '{settings.qdrant_collection_name}'")
            return False

    except Exception as e:
        print(f"✗ Error creating collection: {e}")
        return False


if __name__ == "__main__":
    success = asyncio.run(create_collection())
    if not success:
        exit(1)