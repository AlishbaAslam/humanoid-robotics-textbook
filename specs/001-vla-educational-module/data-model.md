# Data Model: VLA Educational Module

## Entities

### Educational Module
- **name**: string (e.g., "Vision-Language-Action Systems")
- **description**: string (overview of VLA concepts)
- **target_audience**: string (e.g., "Robotics engineers and AI developers")
- **word_count**: integer (between 2000-4000)
- **chapters**: array of Chapter objects
- **status**: enum ("draft", "in_review", "published")

### Chapter
- **title**: string (chapter name)
- **content**: string (Markdown content of the chapter)
- **word_count**: integer (number of words in chapter)
- **technical_concepts**: array of string (key concepts covered)
- **implementation_examples**: array of ImplementationExample objects
- **diagrams**: array of Diagram objects

### Technical Concept
- **name**: string (name of the concept)
- **description**: string (explanation of the concept)
- **references**: array of string (links to official documentation)
- **related_concepts**: array of string (other related concepts)

### Implementation Example
- **title**: string (name of the example)
- **description**: string (what the example demonstrates)
- **code**: string (code snippet or pseudocode)
- **explanation**: string (step-by-step explanation)
- **source**: string (reference to official documentation)

### Diagram
- **title**: string (name of the diagram)
- **description**: string (what the diagram illustrates)
- **type**: enum ("flowchart", "sequence", "architecture", "process")
- **content**: string (Mermaid or other diagram syntax)
- **caption**: string (explanation of the diagram)

### Source Reference
- **title**: string (title of the source)
- **url**: string (URL to the source)
- **publication_date**: string (ISO date format)
- **publisher**: string (e.g., "OpenAI", "ROS", "NVIDIA")
- **verification_status**: enum ("verified", "pending", "invalid")

## Relationships

- Educational Module `1` → `N` Chapters
- Chapter `N` → `M` Technical Concepts
- Chapter `N` → `M` Implementation Examples
- Chapter `N` → `M` Diagrams
- Technical Concept `M` → `N` Technical Concepts (related concepts)
- Implementation Example `M` → `1` Source Reference
- Technical Concept `M` → `N` Source References

## Validation Rules

### Educational Module
- word_count must be between 2000 and 4000
- must have exactly 3 chapters
- target_audience must be "Robotics engineers and AI developers"
- status must be one of the defined enum values

### Chapter
- title must be one of: "Voice-to-Action", "Cognitive Planning", "Capstone Project"
- content must be in Markdown format
- word_count must be positive
- technical_concepts array must not be empty

### Technical Concept
- name must be unique within the module
- references array must not be empty
- publication_date in references must be within the last 5 years

### Implementation Example
- code must be syntactically valid
- source must be a valid URL to official documentation

## State Transitions

### Educational Module
- draft → in_review (when content is complete)
- in_review → published (when reviewed and approved)

## Access Patterns

1. **Module Overview**: Retrieve module metadata and chapter list
2. **Chapter Content**: Retrieve detailed content of a specific chapter
3. **Concept Lookup**: Find information about a specific technical concept
4. **Example Search**: Find implementation examples for specific functionality
5. **Source Verification**: Validate that content is backed by official documentation