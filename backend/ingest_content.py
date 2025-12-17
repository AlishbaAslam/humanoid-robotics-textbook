#!/usr/bin/env python3
"""
Script to ingest website content into Qdrant vector database.
This script fetches content from a Docusaurus site and creates vector embeddings.
"""

import asyncio
import argparse
import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import time
import hashlib
import re
from typing import List, Dict, Any
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from src.config.settings import settings
from src.services.qdrant_service import qdrant_service
from src.services.embedding_service import embedding_service


def fetch_all_urls(base_url: str, max_pages: int = 500) -> List[str]:
    """
    Fetches all discoverable URLs from a Docusaurus site.
    """
    urls = set()
    to_visit = [base_url]
    visited = set()

    # Parse the base domain to only crawl within the same domain
    base_domain = urlparse(base_url).netloc

    while to_visit and len(urls) < max_pages:
        current_url = to_visit.pop(0)

        if current_url in visited:
            continue

        visited.add(current_url)
        urls.add(current_url)

        try:
            # Add a small delay to be respectful to the server
            time.sleep(0.1)

            response = requests.get(current_url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.text, 'html.parser')

            # Find all links in the page
            for link in soup.find_all('a', href=True):
                href = link['href']
                absolute_url = urljoin(current_url, href)

                # Only add URLs from the same domain and that are likely to be content pages
                parsed_url = urlparse(absolute_url)
                if (parsed_url.netloc == base_domain and
                    not absolute_url.endswith(('.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.exe')) and
                    not 'mailto:' in absolute_url and
                    not 'tel:' in absolute_url):

                    # Check if it's a likely content page
                    if (absolute_url.startswith(base_url) and
                        absolute_url not in visited and
                        len(urls) < max_pages):
                        to_visit.append(absolute_url)

        except requests.RequestException as e:
            print(f"Warning: Error fetching {current_url}: {e}")
            continue
        except Exception as e:
            print(f"Warning: Error parsing {current_url}: {e}")
            continue

    return list(urls)


def get_text_from_url(url: str) -> Dict[str, Any]:
    """
    Extracts and cleans text content from a given URL.
    """
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.text, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Try to find the main content area (Docusaurus specific selectors)
        content_selectors = [
            'article',  # Common for content
            'main',     # Main content area
            '.main-wrapper',  # Docusaurus specific
            '.container',     # Common container
            '.theme-doc-markdown',  # Docusaurus markdown content
            '.markdown',      # Markdown content area
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        # If no specific content area found, use the body
        if not content_element:
            content_element = soup.find('body')

        # Extract text from the content element
        if content_element:
            text = content_element.get_text(separator=' ', strip=True)
        else:
            text = soup.get_text(separator=' ', strip=True)

        # Clean up the text (remove extra whitespace, newlines)
        text = re.sub(r'\s+', ' ', text).strip()

        # Extract title
        title_tag = soup.find('title')
        title = title_tag.get_text().strip() if title_tag else "No Title"

        # Calculate content hash
        content_hash = hashlib.sha256(text.encode('utf-8')).hexdigest()

        # Calculate word count
        word_count = len(text.split())

        # Extract additional metadata
        metadata = {}
        description_tag = soup.find('meta', attrs={'name': 'description'})
        if description_tag and description_tag.get('content'):
            metadata['description'] = description_tag.get('content')

        return {
            "url": url,
            "title": title,
            "text": text,
            "word_count": word_count,
            "content_hash": content_hash,
            "metadata": metadata
        }
    except requests.RequestException as e:
        print(f"Error fetching content from {url}: {e}")
        raise
    except Exception as e:
        print(f"Error extracting content from {url}: {e}")
        raise ValueError(f"Content extraction failed for {url}")


def split_text(text: str, max_chunk_size: int = 4000, overlap: int = 200) -> List[Dict[str, Any]]:
    """
    Splits text into smaller chunks while preserving semantic coherence.
    """
    if not text or len(text.strip()) == 0:
        # Return a single empty chunk to ensure every URL contributes at least one vector
        empty_hash = hashlib.sha256("".encode('utf-8')).hexdigest()
        return [{
            "text": "",
            "chunk_index": 0,
            "word_count": 0,
            "content_hash": empty_hash
        }]

    if max_chunk_size <= 0:
        raise ValueError("max_chunk_size must be positive")

    if overlap < 0 or overlap >= max_chunk_size:
        raise ValueError("overlap must be non-negative and less than max_chunk_size")

    chunks = []
    start = 0
    chunk_index = 0

    while start < len(text):
        # Find the end position for this chunk
        end = start + max_chunk_size

        # If we're at the end of the text, use the remaining text
        if end >= len(text):
            end = len(text)
        else:
            # Try to break at sentence boundaries if possible
            # Look for a sentence end (., !, ?) near the end of the chunk
            chunk_text = text[start:end]
            sentence_end_pos = -1

            # Look for sentence endings in the last 200 characters of the chunk
            lookback_range = min(200, len(chunk_text))
            for punct in ['.', '!', '?', '\n']:
                last_punct = chunk_text.rfind(punct, -lookback_range)
                if last_punct > sentence_end_pos:
                    sentence_end_pos = last_punct

            if sentence_end_pos > 0:
                end = start + sentence_end_pos + 1  # Include the punctuation

        # Extract the chunk
        chunk_text = text[start:end]
        chunk_word_count = len(chunk_text.split())
        chunk_hash = hashlib.sha256(chunk_text.encode('utf-8')).hexdigest()

        chunks.append({
            "text": chunk_text,
            "chunk_index": chunk_index,
            "word_count": chunk_word_count,
            "content_hash": chunk_hash
        })

        # Move to the next chunk position (with overlap)
        start = end - overlap if end < len(text) else len(text)
        chunk_index += 1

    return chunks


async def generate_embeddings(chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Generate embeddings for text chunks using Cohere API.
    """
    if not chunks:
        raise ValueError("Chunks list cannot be empty")

    # Process chunks in batches to respect API limits
    batch_size = 96  # Maximum number of texts per Cohere API request
    all_embeddings = []

    for i in range(0, len(chunks), batch_size):
        batch_chunks = chunks[i:i + batch_size]
        batch_texts = [chunk["text"] for chunk in batch_chunks]

        try:
            # Generate embeddings for the batch
            embeddings = await embedding_service.generate_embeddings(batch_texts)

            # Create embedding records with metadata
            for j, embedding_vector in enumerate(embeddings):
                chunk_idx = i + j
                if chunk_idx < len(chunks):  # Safety check
                    chunk = chunks[chunk_idx]
                    all_embeddings.append({
                        "vector": embedding_vector,
                        "text": chunk["text"],
                        "content_hash": chunk["content_hash"],
                        "source_document": chunk.get("url", ""),
                        "title": chunk.get("title", ""),
                        "page_number": chunk.get("chunk_index"),
                        "metadata": chunk.get("metadata", {}),
                        "chunk_id": f"{chunk['content_hash']}_{chunk['chunk_index']}"
                    })

            print(f"Processed batch: {min(i + batch_size, len(chunks))}/{len(chunks)} chunks")

        except Exception as e:
            print(f"Error processing batch starting at chunk {i}: {e}")
            raise

    return all_embeddings


async def ingest_website_content(base_url: str, max_pages: int = 500):
    """Ingest content from a website into Qdrant."""
    print(f"Starting ingestion from: {base_url}")
    print(f"Target collection: {settings.qdrant_collection_name}")
    print(f"Qdrant URL: {settings.qdrant_url}")

    if not settings.qdrant_url:
        print("Error: QDRANT_URL not set in environment variables")
        return False

    if not settings.cohere_api_key:
        print("Error: COHERE_API_KEY not set in environment variables")
        return False

    try:
        # Step 1: Fetch all URLs
        print("Step 1: Fetching all URLs from the site...")
        urls = fetch_all_urls(base_url, max_pages)
        print(f"Found {len(urls)} URLs to process")

        # Step 2: Extract content from each URL
        print("Step 2: Extracting content from URLs...")
        all_chunks = []
        empty_content_count = 0
        for i, url in enumerate(urls):
            print(f"Processing URL {i+1}/{len(urls)}: {url}")
            try:
                content_data = get_text_from_url(url)
                chunks = split_text(content_data["text"], max_chunk_size=4000, overlap=200)

                # Add source metadata to each chunk
                for chunk in chunks:
                    chunk["source_url"] = url
                    chunk["title"] = content_data["title"]
                    chunk["content_hash"] = content_data["content_hash"]

                    # Track if this chunk is empty
                    if not chunk["text"].strip():
                        empty_content_count += 1
                        print(f"DEBUG: URL {url} produced empty content chunk")

                all_chunks.extend(chunks)
            except Exception as e:
                print(f"Error processing URL {url}: {e}")
                # Create a fallback empty chunk to ensure every URL is represented
                empty_hash = hashlib.sha256(f"{url}".encode('utf-8')).hexdigest()
                fallback_chunk = {
                    "text": "",
                    "chunk_index": 0,
                    "word_count": 0,
                    "content_hash": empty_hash,
                    "source_url": url,
                    "title": f"Error processing {url}"
                }
                all_chunks.append(fallback_chunk)
                empty_content_count += 1
                continue

        print(f"Extracted {len(all_chunks)} content chunks from {len(urls)} URLs")
        print(f"Number of chunks with empty content: {empty_content_count}")

        if not all_chunks:
            print("No content chunks extracted. Exiting.")
            return False

        # Step 3: Generate embeddings
        print("Step 3: Generating embeddings...")
        embeddings = await generate_embeddings(all_chunks)
        print(f"Generated embeddings for {len(embeddings)} chunks")

        # Step 4: Create Qdrant collection
        print("Step 4: Creating Qdrant collection...")
        await qdrant_service.create_collection()

        # Step 5: Store embeddings in Qdrant
        print("Step 5: Storing embeddings in Qdrant...")
        result = await qdrant_service.store_embeddings(embeddings)

        print(f"Storage completed. Successful: {result['successful']}, Failed: {result['failed']}")

        if result['errors']:
            print(f"Storage errors: {result['errors']}")

        print(f"✓ Ingestion pipeline completed successfully!")
        print(f"✓ Collection '{settings.qdrant_collection_name}' is ready for queries!")

        # Check how many points are in the collection now
        count = await qdrant_service.count_points()
        print(f"✓ Collection now contains {count} points")

        return True

    except Exception as e:
        print(f"✗ Pipeline failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(description='Ingest website content into Qdrant')
    parser.add_argument('--url', type=str, required=True, help='Base URL of the website to ingest')
    parser.add_argument('--max-pages', type=int, default=500, help='Maximum number of pages to crawl (default: 500)')
    parser.add_argument('--chunk-size', type=int, default=4000, help='Maximum character size for text chunks (default: 4000)')
    parser.add_argument('--overlap', type=int, default=200, help='Character overlap between chunks (default: 200)')

    args = parser.parse_args()

    if not args.url.startswith(('http://', 'https://')):
        print("Error: URL must start with http:// or https://")
        return 1

    success = asyncio.run(ingest_website_content(args.url, args.max_pages))
    return 0 if success else 1


if __name__ == "__main__":
    exit(main())