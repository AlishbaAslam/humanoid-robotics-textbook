# Data Model: Retrieval Pipeline Testing for RAG Chatbot

## Entities

### Query
- **Description**: A natural language search request from a user that needs to be semantically matched against stored content
- **Fields**:
  - `text`: string - The natural language query text
  - `id`: string - Unique identifier for the query
  - `timestamp`: datetime - When the query was submitted
  - `expected_results`: list - Optional field for validation of expected outcomes

### Text Chunk
- **Description**: A segment of book content that has been embedded and stored in the vector database, containing the actual text and associated metadata
- **Fields**:
  - `text`: string - The actual content text
  - `chunk_index`: integer - Position of the chunk within the original document
  - `word_count`: integer - Number of words in the chunk
  - `content_hash`: string - SHA256 hash of the content for uniqueness
  - `source_url`: string - URL of the original document
  - `title`: string - Title of the original document
  - `vector`: list[float] - Embedding vector representation

### Metadata
- **Description**: Information about the source of the text chunk including URL, section, chunk identifier, and other provenance data
- **Fields**:
  - `source_url`: string - Original URL where the content was found
  - `title`: string - Title of the document containing the chunk
  - `chunk_id`: string - Unique identifier for this chunk
  - `content_hash`: string - Hash of the content for verification
  - `original_chunk_index`: integer - Original position in the document

### Similarity Score
- **Description**: A numerical value representing how semantically relevant a text chunk is to the input query
- **Fields**:
  - `score`: float - Similarity score between 0 and 1 (or appropriate range for the similarity function)
  - `query_id`: string - Reference to the query that generated this score
  - `chunk_id`: string - Reference to the chunk that was scored
  - `rank`: integer - Position in the ranked results list

### Retrieval Result
- **Description**: A container for the results of a retrieval operation, including the matched chunks and associated metadata
- **Fields**:
  - `query`: Query - The original query that was processed
  - `results`: list[Text Chunk] - List of matching text chunks, ranked by relevance
  - `metadata_list`: list[Metadata] - Metadata for each result
  - `similarity_scores`: list[Similarity Score] - Scores for each result
  - `retrieval_time`: float - Time taken to perform the retrieval operation
  - `total_candidates`: integer - Total number of candidates considered
  - `filtered_by_threshold`: integer - Number of results filtered by similarity threshold

### Test Case
- **Description**: A structured definition of a test to validate the retrieval pipeline
- **Fields**:
  - `name`: string - Descriptive name for the test case
  - `query_text`: string - The query to be tested
  - `expected_urls`: list[string] - Expected source URLs in the results
  - `expected_keywords`: list[string] - Keywords expected to appear in results
  - `min_similarity`: float - Minimum acceptable similarity score
  - `max_response_time`: float - Maximum acceptable response time in seconds
  - `num_results_expected`: integer - Expected number of results to return

### Test Result
- **Description**: The outcome of executing a test case against the retrieval pipeline
- **Fields**:
  - `test_case`: Test Case - Reference to the test case that was executed
  - `retrieval_result`: Retrieval Result - The actual results from the retrieval
  - `passed`: boolean - Whether the test case passed or failed
  - `execution_time`: float - Time taken to execute the test
  - `details`: string - Detailed information about the test execution
  - `similarity_accuracy`: float - Accuracy score for semantic relevance
  - `metadata_accuracy`: float - Accuracy score for metadata preservation