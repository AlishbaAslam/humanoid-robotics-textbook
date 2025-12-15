# Data Model: ROS2 Middleware Educational Module

## Content Entities

### Module
- **id**: string (e.g., "module-1-ros2")
- **title**: string (e.g., "The Robotic Nervous System (ROS 2)")
- **description**: string (brief overview of the module)
- **target_audience**: string (e.g., "Beginner to intermediate robotics developers and AI engineers")
- **prerequisites**: array of strings (skills/knowledge needed)
- **learning_objectives**: array of strings (what learners will achieve)
- **duration**: string (estimated time to complete)
- **chapters**: array of Chapter references
- **exercises**: array of Exercise references

### Chapter
- **id**: string (e.g., "chapter-1-ros2")
- **module_id**: string (reference to parent module)
- **title**: string (e.g., "Fundamentals of ROS 2 Nodes, Topics, and Services")
- **subtitle**: string (optional descriptive subtitle)
- **content**: string (the complete chapter content in Markdown format, including exercises)
- **learning_objectives**: array of strings (specific objectives for this chapter)
- **prerequisites**: array of strings (specific prerequisites for this chapter)
- **duration**: string (estimated time to complete this chapter)
- **sections**: array of Section references
- **code_examples**: array of CodeExample references
- **exercises**: array of Exercise references (embedded within the content)
- **citations**: array of Citation references
- **diagrams**: array of Diagram references

### Section
- **id**: string (e.g., "section-nodes-introduction")
- **chapter_id**: string (reference to parent chapter)
- **title**: string (e.g., "Introduction to ROS 2 Nodes")
- **content**: string (the section content in Markdown format)
- **order**: integer (position within the chapter)
- **type**: string (e.g., "concept", "example", "exercise", "summary")

### CodeExample
- **id**: string (e.g., "example-basic-publisher")
- **chapter_id**: string (reference to parent chapter)
- **title**: string (e.g., "Basic Publisher Node")
- **description**: string (what the example demonstrates)
- **language**: string (e.g., "python", "urdf", "bash")
- **code**: string (the actual code content)
- **file_path**: string (where the code file is stored)
- **expected_output**: string (what output to expect when running)
- **execution_instructions**: string (how to run the example)
- **related_concepts**: array of strings (concepts demonstrated by this example)

### Exercise
- **id**: string (e.g., "exercise-publisher-subscriber")
- **chapter_id**: string (reference to parent chapter)
- **title**: string (e.g., "Create a Publisher-Subscriber Pair")
- **description**: string (what the exercise requires)
- **instructions**: string (step-by-step instructions)
- **success_criteria**: array of strings (how to know the exercise is complete)
- **difficulty**: string (e.g., "beginner", "intermediate", "advanced")
- **estimated_time**: string (how long the exercise should take)
- **related_concepts**: array of strings (concepts practiced in this exercise)
- **solution**: string (optional solution for reference)

### Citation
- **id**: string (e.g., "citation-ros2-concepts")
- **chapter_id**: string (reference to parent chapter, or null for module-level)
- **title**: string (e.g., "ROS 2 Concepts - ROS Wiki")
- **url**: string (the URL to the cited resource)
- **access_date**: string (when the resource was accessed, ISO format)
- **type**: string (e.g., "documentation", "tutorial", "paper", "official")
- **relevance**: string (why this citation is relevant to the content)

### Diagram
- **id**: string (e.g., "diagram-node-communication")
- **chapter_id**: string (reference to parent chapter)
- **title**: string (e.g., "Node Communication Pattern")
- **description**: string (what the diagram illustrates)
- **type**: string (e.g., "mermaid", "plantuml", "image", "ascii")
- **content**: string (the diagram definition or path to image)
- **alt_text**: string (alternative text for accessibility)
- **related_concepts**: array of strings (concepts illustrated by this diagram)

### AIChatbotIntegration
- **id**: string (e.g., "chatbot-ros2-module")
- **module_id**: string (reference to the module)
- **knowledge_base_path**: string (path to the vector database for this module)
- **triggers**: array of strings (keywords/phrases that activate specific responses)
- **context_window**: integer (how much context to provide to the AI)
- **response_templates**: array of ResponseTemplate references

### ResponseTemplate
- **id**: string (e.g., "template-code-explanation")
- **chatbot_integration_id**: string (reference to parent integration)
- **trigger_keyword**: string (what keyword triggers this template)
- **template**: string (the response template with placeholders)
- **context_requirements**: array of strings (what context is needed for this response)

## Relationships

### Module contains:
- 1..* Chapters
- 0..* Exercises (module-level exercises)

### Chapter contains:
- 1..* Sections
- 0..* CodeExamples
- 0..* Exercises (chapter-specific)
- 0..* Citations
- 0..* Diagrams

### Section belongs to:
- Exactly 1 Chapter

### CodeExample belongs to:
- Exactly 1 Chapter

### Exercise belongs to:
- Exactly 1 Chapter (or 1 Module for module-level exercises)

### Citation belongs to:
- Exactly 1 Chapter (or null for module-level citations)

### Diagram belongs to:
- Exactly 1 Chapter

### AIChatbotIntegration belongs to:
- Exactly 1 Module
- Contains 0..* ResponseTemplates

## Validation Rules

1. **Module Validation**:
   - Must have at least one chapter
   - Title and description are required
   - Learning objectives must be specific and measurable

2. **Chapter Validation**:
   - Must have at least one section
   - Title and content are required
   - Learning objectives must align with module objectives
   - Must include at least one code example or exercise

3. **CodeExample Validation**:
   - Code content and file_path are required
   - Language must be valid (python, urdf, bash, etc.)
   - Must have execution instructions

4. **Exercise Validation**:
   - Title, description, and instructions are required
   - Success criteria must be specific and verifiable
   - Difficulty level must be specified

5. **Citation Validation**:
   - Title and URL are required
   - URL must be valid and accessible
   - Access date must be recent (within 6 months)

6. **Diagram Validation**:
   - Title and content are required
   - Alt text must be provided for accessibility

## Content State Transitions

### Content Creation Workflow:
1. **Draft**: Content is being created, not yet reviewed
2. **Reviewed**: Content has been reviewed by subject matter expert
3. **Validated**: Content has been tested and verified for accuracy
4. **Published**: Content is available to learners

### State Transition Rules:
- Draft → Reviewed: Requires peer review and approval
- Reviewed → Validated: Requires technical validation and testing
- Validated → Published: Requires final quality check

## Indexing Strategy

### For AI Chatbot:
- Module and chapter titles for topic-based queries
- Section headings for specific concept queries
- Code example descriptions for technical queries
- Exercise descriptions for practical queries
- Citation content for reference queries

### For Search:
- Full-text search on content fields
- Tag-based search on related_concepts
- Difficulty-based filtering for exercises
- Type-based filtering (concept vs example vs exercise)

This data model provides a structured approach to organizing the educational content while maintaining flexibility for the AI integration and search capabilities required by the project.