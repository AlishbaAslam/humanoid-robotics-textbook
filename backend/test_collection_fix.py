#!/usr/bin/env python3
"""
Test script to verify the Qdrant collection name fix
"""

import asyncio
from src.config.settings import settings
from src.services.qdrant_service import qdrant_service


async def test_collection_name():
    """Test that the collection name is consistent."""
    print("Testing Qdrant collection name configuration...")
    print(f"Settings collection name: {settings.qdrant_collection_name}")
    print(f"Qdrant service collection name: {qdrant_service.collection_name}")

    # Check if collection exists
    exists = await qdrant_service.verify_collection()
    print(f"Collection '{qdrant_service.collection_name}' exists: {exists}")

    if exists:
        count = await qdrant_service.count_points()
        print(f"Collection contains {count} points")

    return settings.qdrant_collection_name == qdrant_service.collection_name


if __name__ == "__main__":
    success = asyncio.run(test_collection_name())
    if success:
        print("✓ Collection names are consistent!")
    else:
        print("✗ Collection names are NOT consistent!")
        exit(1)