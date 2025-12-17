# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/1-textbook-repo-skeleton/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by phases and user stories to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **textbook-site**: `textbook-site/`
- **RAG Backend**: `rag-backend/`
- **Bonus Features**: `bonus-features/`

---

## Phase 1: Project Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure. Corresponds to User Stories 1 and 3.

- [ ] T001 Verify Docusaurus v3 installation and dependencies in `textbook-site/package.json`.
- [x] T002 [P] Create chapter directory structure in `textbook-site/docs/` for: `00-introduction`, `01-ros2`, `02-digital-twin`, `03-isaac`, `04-vla`, `05-capstone`, `06-conclusion`.
- [x] T003 [P] Create `assets` directory in `textbook-site/static/` with subdirectories for `diagrams` and `videos`.
- [x] T004 [P] Create `rag-backend` directory at the project root with placeholder files: `main.py`, `database.py`, `models.py`, `schemas.py`, `vector_db.py`, and an `ingestion` subdirectory.
- [x] T005 [P] Create `bonus-features` directory at the project root with subdirectories for `subagents`, `authentication`, `personalization`, and `translation`.
- [ ] T006 Create placeholder GitHub Actions workflow in `.github/workflows/deploy.yml` for GitHub Pages deployment.

---

## Phase 2: Foundational Content Creation

**Purpose**: Create the empty chapter files. Corresponds to User Story 2.

- [x] T007 [P] [US2] Create placeholder chapter file `textbook-site/docs/00-introduction/intro.md`.
- [x] T008 [P] [US2] Create placeholder chapter file `textbook-site/docs/01-ros2/ros2-basics.md`.
- [x] T009 [P] [US2] Create placeholder chapter file `textbook-site/docs/01-ros2/ros2-advanced.md`.
- [x] T010 [P] [US2] Create placeholder chapter file `textbook-site/docs/02-digital-twin/physics-simulation.md`.
- [x] T011 [P] [US2] Create placeholder chapter file `textbook-site/docs/02-digital-twin/sensor-integration.md`.
- [x] T012 [P] [US2] Create placeholder chapter file `textbook-site/docs/03-isaac/perception-vslam.md`.
- [x] T013 [P] [US2] Create placeholder chapter file `textbook-site/docs/03-isaac/navigation-sim-to-real.md`.
- [x] T014 [P] [US2] Create placeholder chapter file `textbook-site/docs/04-vla/voice-to-action.md`.
- [x] T015 [P] [US2] Create placeholder chapter file `textbook-site/docs/04-vla/llm-cognitive-planning.md`.
- [x] T016 [P] [US2] Create placeholder chapter file `textbook-site/docs/04-vla/conversational-robotics.md`.
- [x] T017 [P] [US2] Create placeholder chapter file `textbook-site/docs/05-capstone/end-to-end-project.md`.
- [x] T018 [P] [US2] Create placeholder chapter file `textbook-site/docs/06-conclusion/summary-future-trends.md`.
- [x] T019 [US2] Update `sidebars.ts` in `textbook-site/` to include all new chapters.

---

## Phase 3: RAG Backend Setup

**Purpose**: Establish the skeleton for the RAG backend. Corresponds to User Story 4.

- [x] T020 [US4] Implement basic FastAPI app structure in `rag-backend/main.py`.
- [x] T021 [US4] Define placeholder schema for Neon/PostgreSQL in `rag-backend/database.py`.
- [x] T022 [US4] Configure Qdrant client placeholder in `rag-backend/vector_db.py`.
- [x] T023 [US4] Create content ingestion script placeholder in `rag-backend/ingestion/ingest.py`.

---

## Phase 4: Bonus Features Integration

**Purpose**: Create placeholders for future bonus feature implementations.

- [x] T024 [P] Create Claude Subagents placeholder `bonus-features/subagents/main.py`.
- [x] T025 [P] Create Better-Auth signup/quiz placeholder `bonus-features/authentication/main.py`.
- [x] T026 [P] Create Personalization logic placeholder `bonus-features/personalization/main.py`.
- [x] T027 [P] Create Urdu Translation toggle placeholder `bonus-features/translation/main.py`.

---

## Phase 5: Testing & Deployment

**Purpose**: Validate the features and deploy the textbook.

- [x] T028 Create benchmark query test script placeholder for RAG in `rag-backend/tests/test_rag.py`.
- [x] T029 Perform a manual WCAG accessibility check on the deployed Docusaurus site.
- [x] T030 Finalize and enable the deployment workflow in `.github/workflows/deploy.yml`.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational Content Creation (Phase 2)**: Depends on Setup completion.
- **RAG Backend Setup (Phase 3)**: Depends on Setup completion.
- **Bonus Features Integration (Phase 4)**: Depends on Setup completion.
- **Testing & Deployment (Phase 5)**: Depends on all other phases being complete.

### Parallel Opportunities

- Most tasks in Phase 1 can be run in parallel.
- All content creation tasks in Phase 2 can be run in parallel after the directory structure is created.
- The RAG backend, and bonus features phases can be worked on in parallel with content creation.