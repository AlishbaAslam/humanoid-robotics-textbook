---
description: "Task list for Docusaurus Homepage implementation"
---

# Tasks: Docusaurus Homepage for Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-docusaurus-homepage/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No tests requested in feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `website/src/` at repository root
- **Homepage**: `website/src/pages/index.js` for main page
- **Components**: `website/src/components/` for reusable components
- **Styles**: `.module.css` files for component-specific styling

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Verify current homepage structure by examining website/src/pages/index.js
- [x] T002 Verify existing HomepageFeatures component in website/src/components/HomepageFeatures/index.js
- [x] T003 [P] Review current styling in website/src/pages/index.module.css
- [x] T004 [P] Review current component styling in website/src/components/HomepageFeatures/styles.module.css

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Update hero section in website/src/pages/index.js with exact title and subtitle
- [x] T006 [P] Update hero styling in website/src/pages/index.module.css to ensure background #2E8555
- [x] T007 [P] Update CTA button in website/src/pages/index.js to link to /docs/introduction
- [x] T008 Update module cards structure in website/src/components/HomepageFeatures/index.js to match spec requirements

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Homepage Discovery (Priority: P1) 🎯 MVP

**Goal**: Create a professional, visually appealing homepage with hero section that displays title, subtitle, and CTA button

**Independent Test**: Can be fully tested by visiting the homepage and verifying that it displays a professional hero section with title "Physical AI & Humanoid Robotics Textbook" with the subtitle "A textbook covering Physical AI, Robotics, Control, Simulation, and Real-World Embodiment" on a green background with white text, and a "Start Reading" button that links to /docs/introduction

### Implementation for User Story 1

- [x] T009 [US1] Verify hero section displays correct title "Physical AI & Humanoid Robotics Textbook" in website/src/pages/index.js
- [x] T010 [US1] Verify hero section displays correct subtitle "A textbook covering Physical AI, Robotics, Control, Simulation, and Real-World Embodiment" in website/src/pages/index.js
- [x] T011 [US1] Verify hero section has background color #2E8555 and white text in website/src/pages/index.module.css
- [x] T012 [US1] Verify CTA button text is "Start Reading" and links to /docs/introduction in website/src/pages/index.js
- [x] T013 [US1] Test hero section appearance and functionality across different screen sizes

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Module Exploration (Priority: P1)

**Goal**: Display all available modules clearly on the homepage in an organized grid with equal-sized, outlined cards that have hover effects

**Independent Test**: Can be fully tested by viewing the modules section and verifying that all modules are displayed as equal-sized cards with outlines, with hover effects that highlight the outline color to #276944 and slightly zoom the card

### Implementation for User Story 2

- [x] T014 [US2] Update module cards in website/src/components/HomepageFeatures/index.js to ensure all cards are equal in size
- [x] T015 [US2] Add visible outline/border to all module cards in website/src/components/HomepageFeatures/styles.module.css
- [x] T016 [US2] Implement hover effect that highlights outline color to #276944 in website/src/components/HomepageFeatures/styles.module.css
- [x] T017 [US2] Implement hover effect that slightly zooms the card (scale 1.05) in website/src/components/HomepageFeatures/styles.module.css
- [x] T018 [US2] Add smooth transition for hover effects in website/src/components/HomepageFeatures/styles.module.css
- [x] T019 [US2] Verify all module cards have "Learn More" button linking to correct documentation paths in website/src/components/HomepageFeatures/index.js
- [x] T020 [US2] Test hover effects work consistently across all module cards

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Mobile-Friendly Access (Priority: P2)

**Goal**: Ensure the homepage is responsive and maintains all functionality across different screen sizes (mobile, tablet, desktop)

**Independent Test**: Can be fully tested by viewing the homepage on different screen sizes and verifying responsive behavior while maintaining all functionality

### Implementation for User Story 3

- [x] T021 [US3] Verify responsive grid layout works on mobile devices in website/src/components/HomepageFeatures/styles.module.css
- [x] T022 [US3] Ensure hover effects are touch-friendly on mobile devices in website/src/components/HomepageFeatures/styles.module.css
- [x] T023 [US3] Test layout adaptation on tablet screen sizes in website/src/components/HomepageFeatures/styles.module.css
- [x] T024 [US3] Verify all links and functionality work on mobile screen sizes
- [x] T025 [US3] Test that module cards maintain equal size across all screen sizes

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T026 [P] Verify footer remains unchanged and functional
- [x] T027 [P] Test overall page load performance and responsiveness
- [x] T028 [P] Validate consistent green (#2E8555) and white theme across all sections
- [x] T029 [P] Verify all links navigate to correct documentation pages without errors
- [x] T030 Run quickstart.md validation to ensure all functionality works as expected

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May build on US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May build on US1/US2 but should be independently testable

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 2

```bash
# Launch all styling tasks for User Story 2 together:
Task: "Add visible outline/border to all module cards in website/src/components/HomepageFeatures/styles.module.css"
Task: "Implement hover effect that highlights outline color to #276944 in website/src/components/HomepageFeatures/styles.module.css"
Task: "Implement hover effect that slightly zooms the card (scale 1.05) in website/src/components/HomepageFeatures/styles.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify functionality after each task or logical group
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence