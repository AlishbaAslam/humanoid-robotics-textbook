# Quickstart: Docusaurus Homepage for Physical AI & Humanoid Robotics Textbook

## Development Setup

1. **Prerequisites**
   - Node.js 18+
   - npm or yarn
   - Git

2. **Installation**
   ```bash
   cd website
   npm install
   ```

3. **Local Development**
   ```bash
   cd website
   npm start
   ```
   This command starts a local development server and opens the website in your browser. Most changes are reflected live without requiring a restart.

## Key Files to Modify

### Homepage Structure
- `website/src/pages/index.js` - Main homepage component with hero section
- `website/src/pages/index.module.css` - Hero section styling

### Module Cards
- `website/src/components/HomepageFeatures/index.js` - Module cards component
- `website/src/components/HomepageFeatures/styles.module.css` - Module cards styling

## Implementation Steps

1. **Update Hero Section** (`index.js`):
   - Set title to "Physical AI & Humanoid Robotics Textbook"
   - Set subtitle to "A textbook covering Physical AI, Robotics, Control, Simulation, and Real-World Embodiment"
   - Update CTA button to "Start Reading" linking to `/docs/physical-ai-intro/introduction`
   - Ensure background color is #2E8555 with white text

2. **Style Module Cards** (`HomepageFeatures/styles.module.css`):
   - Ensure all cards are equal size
   - Add visible border with #2E8555
   - Implement hover effects: border highlight to #276944 and slight zoom (scale 1.05)
   - Add smooth transitions for hover effects

3. **Test Responsiveness**:
   - Verify layout on mobile, tablet, and desktop
   - Ensure hover effects work appropriately (disable scaling on mobile)

## Running Tests

```bash
# Build the website
cd website
npm run build

# Serve the built website locally
npm run serve
```

## Deployment

The website can be deployed to GitHub Pages, Netlify, Vercel, or any static hosting service. For GitHub Pages specifically:

1. Build the website: `npm run build`
2. The static files will be generated in the `build` directory
3. Configure your hosting platform to serve from the `build` directory