# Data Model: Docusaurus Homepage for Physical AI & Humanoid Robotics Textbook

## Entities

### Homepage Content
- **Fields**:
  - title: string (e.g., "Physical AI & Humanoid Robotics Textbook")
  - subtitle: string (e.g., "A textbook covering Physical AI, Robotics, Control, Simulation, and Real-World Embodiment")
  - ctaText: string (e.g., "Start Reading")
  - ctaLink: string (e.g., "/docs/introduction")
  - backgroundColor: string (e.g., "#2E8555")
  - textColor: string (e.g., "white")

### Module Information
- **Fields**:
  - title: string (e.g., "Module 1: The Robotic Nervous System (ROS 2)")
  - path: string (e.g., "/docs/module-1-ros2/intro")
  - description: string (text content for the module)
  - keyPoints: array of strings (bullet points highlighting key features)
  - outlineColor: string (e.g., "#2E8555")
  - hoverColor: string (e.g., "#276944")

## Relationships
- Homepage Content contains multiple Module Information entities
- Each Module Information entity has a relationship to documentation pages via the path field

## Validation Rules
- Homepage title must be non-empty
- Module titles must be unique
- Module paths must be valid internal links
- Colors must be valid CSS color values
- All modules must be displayed in responsive grid layout

## State Transitions
- No state transitions required (static content display)