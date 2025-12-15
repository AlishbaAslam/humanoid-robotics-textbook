# Content Contracts: Docusaurus Homepage for Physical AI & Humanoid Robotics Textbook

## Contract Overview

This contract defines the content structure and API endpoints for the homepage of the Physical AI & Humanoid Robotics textbook website.

## Homepage Content Structure

### Hero Section
```
{
  "title": "Physical AI & Humanoid Robotics Textbook",
  "subtitle": "A textbook covering Physical AI, Robotics, Control, Simulation, and Real-World Embodiment",
  "ctaText": "Start Reading",
  "ctaLink": "/docs/physical-ai-intro/introduction",
  "backgroundColor": "#2E8555",
  "textColor": "white"
}
```

### Module Card Structure
```
{
  "title": "string",
  "path": "string (valid internal documentation link)",
  "description": "string (content description)",
  "keyPoints": "array of strings",
  "outlineColor": "string (CSS color value)",
  "hoverColor": "string (CSS color value)"
}
```

## Validation Rules

1. All content fields must be non-empty
2. All paths must be valid internal links
3. All color values must be valid CSS color formats
4. All module cards must have equal dimensions when rendered
5. Hover effects must be consistent across all cards

## Expected Behaviors

1. Hero section must render with exact background color #2E8555
2. All module cards must have visible outlines
3. Hover effects must include border highlight and slight zoom
4. All links must navigate to correct documentation pages
5. Layout must be responsive across all screen sizes

## Success Criteria

1. Homepage loads within 2 seconds
2. All elements render correctly on first page load
3. Hover effects work consistently across supported browsers
4. Mobile responsiveness is maintained
5. All links navigate to correct destinations