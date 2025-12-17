# Implementation Plan: Textbook Repository Skeleton

**Branch**: `1-textbook-repo-skeleton` | **Date**: 2025-12-07 | **Spec**: ../spec.md
**Input**: Feature specification from `/specs/1-textbook-repo-skeleton/spec.md`

## Summary

This plan outlines the detailed strategy for developing the "Physical AI & Humanoid Robotics Textbook (Docusaurus Project)". It encompasses a multi-faceted approach to build a Docusaurus v3-based textbook skeleton, integrate a RAG (Retrieval-Augmented Generation) backend, and incorporate bonus features such as authentication, personalization, and Urdu translation. The core technical approach prioritizes academic rigor, reproducibility, and deployability via GitHub Pages, aligning with the curriculum focus on embodied intelligence.

## Technical Context

**Language/Version**: Python 3.10+ (for RAG backend), JavaScript (Node.js 18+ for Docusaurus)  
**Primary Dependencies**: Docusaurus v3, npm/yarn, FastAPI, Qdrant, Neon (PostgreSQL), GitHub Pages, Better-Auth (placeholder), Spec-Kit Plus, Git, GitHub Actions.  
**Storage**: Qdrant (vector DB for RAG embeddings), Neon (PostgreSQL for RAG metadata, user preferences for personalization).  
**Testing**: Pytest (for RAG backend unit/integration tests), Docusaurus `build` and `serve` commands (for frontend validation), GitHub Actions CI (for automated build/deploy tests), manual user flow testing (for personalized content, Urdu translation, auth, RAG accuracy).  
**Target Platform**: Frontend: Static web (Docusaurus deployed on GitHub Pages). Backend: Containerized (Docker/Kubernetes for FastAPI, Qdrant, Neon).  
**Project Type**: Full-stack web application, comprising a static content frontend (Docusaurus) and a dynamic API backend (FastAPI) for RAG and future services.  
**Performance Goals**: RAG accuracy ≥90% over 20 benchmark queries (defined in Quality Validation), Docusaurus build time <5 minutes, frontend chapter load time <3 seconds on average.  
**Constraints**: WCAG-accessible design, MIT License for the project, preference for free-tier cloud services where applicable, entirely simulation-focused (no hardware required for core content), APA style citations for all academic references.  
**Scale/Scope**: The project will deliver a comprehensive textbook with 10+ chapters organized into 4 major modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA), plus an Introduction and Conclusion. It includes a RAG backend skeleton, Auth, Personalization, and Urdu Translation module placeholders.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The plan aligns with the project's constitution:

-   **Accuracy through primary source verification**: The plan explicitly includes "Minimum 5 peer-reviewed academic sources per module" and "Every factual claim must be traceable", directly supporting this principle.
-   **Clarity for an academic audience (computer science background)**: The project's target audience and the detailed technical context and architecture design ensure alignment with this principle.
-   **Reproducibility (all claims cited and traceable)**: APA citation rules and traceability are embedded in the research approach, supporting reproducibility.
-   **Rigor (peer-reviewed sources preferred)**: The research approach prioritizes peer-reviewed academic sources, fulfilling the rigor principle.

All principles are adhered to.

## Architecture

The project follows a decoupled, modular architecture:

```
+-----------------------------------+
|     GitHub Pages (Frontend)       |
|                                   |
|   +---------------------------+   |
|   |    Docusaurus v3 Site     |   |
|   |  (Static Content + UI)    |   |
|   |                           |   |
|   |   - Chapters (MDX)        |   |
|   |   - Sidebars, Config      |   |
|   |   - Custom React Comp.    |   |
|   |     (Personalization Btn) |   |
|   |     (Urdu Translation Btn)|   |
|   +--------------|------------+   |
|                  |                |
+------------------|----------------+
                   | (API Calls)
                   |
+------------------v----------------+
|     Cloud / Backend Services      |
|                                   |
|   +---------------------------+   |
|   |       API Gateway /       |   |
|   |       Load Balancer       |   |
|   +--------------|------------+   |
|                  |                |
|   +--------------v------------+   |
|   |       FastAPI App         |   |
|   |     (RAG Backend)         |   |
|   |   - Search Endpoints      |   |
|   |   - Ingestion Endpoints   |   |
|   |   - Better-Auth Layer     |   |
|   |   - Personalization Logic |   |
|   +--------------|------------+   |
|                  |                |
|    +-------------v------------+   +-----------------+
|    |     Vector Database      |     |  SQL Database   |
|    |      (Qdrant)            |     |  (Neon/PostgreSQL)|
|    |   - Embeddings          |     |  - RAG Metadata   |
|    |   - Vector Search       |     |  - User Data      |
|    +--------------------------+     +-----------------+
+-----------------------------------+
```

**Component Interactions:**
*   **Docusaurus Site**: Serves static textbook content. Custom React components make API calls to the FastAPI backend for dynamic features like RAG queries, personalization logic, and potentially triggering Urdu translation services (if external).
*   **FastAPI App (RAG Backend)**: Acts as the central API for all dynamic features. It handles RAG queries by interacting with Qdrant for vector search and Neon for metadata. It also houses the Better-Auth integration and personalization logic, using Neon for user data.
*   **Qdrant**: Stores vector embeddings of textbook content for efficient semantic search.
*   **Neon (PostgreSQL)**: Serves as the relational database for RAG metadata (e.g., chapter IDs, section titles associated with embeddings) and user-specific data required for authentication and personalization features.
*   **Content Storage + Embeddings Flow**: Textbook content (MDX files) is ingested. A background process or API endpoint will extract text, generate embeddings using a suitable LLM, and store them in Qdrant, along with relevant metadata in Neon.

## Section Structure

The textbook will be structured into 4 major modules, each containing 2-3 chapters, plus an introductory and a concluding chapter, totaling 10+ chapters.

*   **Introduction (Chapter 00):**
    *   Learning Objectives: Understand the motivation behind Physical AI, overview of the textbook's content, target audience, and prerequisites.
    *   Key Concepts: Embodied Intelligence, AI-native software, Humanoid Robotics.
    *   [Personalization Button Placeholder]
    *   [Urdu Translation Button Placeholder]

*   **Module 1 — ROS 2 (Robotic Nervous System):**
    *   Chapters: 01-ROS2 Basics, 02-ROS2 Advanced (URDF, Python Agents)
    *   Each chapter will include:
        *   Clear learning objectives (e.g., "Understand ROS 2 node communication," "Model a robot with URDF").
        *   Key concepts (e.g., "Nodes," "Topics," "Services," "URDF semantics").
        *   [Personalization Button Placeholder]
        *   [Urdu Translation Button Placeholder]

*   **Module 2 — Gazebo & Unity (Digital Twin):**
    *   Chapters: 03-Physics Simulation, 04-Sensor Integration & Environments
    *   Each chapter will include:
        *   Clear learning objectives (e.g., "Set up Gazebo/Unity physics simulation," "Integrate virtual LiDAR/IMU sensors").
        *   Key concepts (e.g., "Physics engines," "Sensor models," "Simulation environments").
        *   [Personalization Button Placeholder]
        *   [Urdu Translation Button Placeholder]

*   **Module 3 — NVIDIA Isaac (AI-Robot Brain):**
    *   Chapters: 05-Perception & VSLAM, 06-Navigation & Sim-to-Real
    *   Each chapter will include:
        *   Clear learning objectives (e.g., "Implement VSLAM algorithms," "Transfer policies from simulation to real robots").
        *   Key concepts (e.g., "Computer Vision," "SLAM," "Reinforcement Learning in simulation").
        *   [Personalization Button Placeholder]
        *   [Urdu Translation Button Placeholder]

*   **Module 4 — Vision-Language-Action (VLA):**
    *   Chapters: 07-Voice-to-Action, 08-LLM Cognitive Planning, 09-Conversational Robotics
    *   Each chapter will include:
        *   Clear learning objectives (e.g., "Design speech interfaces for robots," "Use LLMs for high-level robot task planning").
        *   Key concepts (e.g., "Speech Recognition," "Natural Language Understanding," "Cognitive Architectures").
        *   [Personalization Button Placeholder]
        *   [Urdu Translation Button Placeholder]

*   **Capstone (Chapter 10+):**
    *   Learning Objectives: Integrate all learned concepts into an end-to-end autonomous humanoid system.
    *   Key Concepts: System integration, Voice command → Perception → Navigation → Object Manipulation.
    *   [Personalization Button Placeholder]
    *   [Urdu Translation Button Placeholder]

*   **Conclusion (Final Chapter):**
    *   Learning Objectives: Summarize key takeaways, discuss future trends in Physical AI and Humanoid Robotics.
    *   Key Concepts: Future directions, ethical considerations (briefly).
    *   [Personalization Button Placeholder]
    *   [Urdu Translation Button Placeholder]

## Research Approach

A mixed research methodology will be employed, emphasizing **concurrent writing and resource gathering**. This iterative approach allows for immediate integration of new findings while continuously refining existing content.

-   **Concurrent Writing + Resource Gathering**: Content creation will proceed alongside active research. As sections are drafted, relevant primary and secondary sources will be identified and integrated.
-   **Minimum 5 Peer-Reviewed Academic Sources per Module**: Each of the 4 major modules (ROS 2, Digital Twin, NVIDIA Isaac, VLA) will require a minimum of 5 peer-reviewed academic sources to substantiate claims and provide foundational knowledge.
-   **Every Factual Claim Must Be Traceable**: All data, figures, and technical assertions will be backed by precise citations.
-   **APA Citation Rules**: All references and in-text citations will strictly follow APA style guidelines, as defined in the project constitution.

## Quality Validation

Measurable validation targets will ensure the project meets its objectives:

-   **RAG Accuracy Target**: Achieve an accuracy of **≥90%** for the RAG backend over a benchmark set of 20 domain-specific queries. This will be measured by evaluating the relevance and correctness of the retrieved information and generated responses.
-   **Full User Flow Test After Deployment**: Conduct an end-to-end test of the primary user journey post-deployment:
    `signup → personalized chapter experience → Urdu translation toggle → interactive quiz (placeholder)`
    This validates core features and their integration.
-   **Simulated User Interactions for Personalization + Subagent Features**: Implement simulated user scenarios to test the responsiveness and effectiveness of personalization logic and any planned subagent functionalities, ensuring they provide meaningful value.

## Implementation Phases

The project will be executed in 4 distinct phases, with clear deliverables:

1.  **Phase 1: Core Textbook Skeleton (Weeks 1-2)**
    *   **Deliverables**: Complete Docusaurus v3 project setup, initial chapter structure (10+ empty MDX stubs with front-matter), `specs/` directory with `spec.md`, `plan.md`, `tasks.md`, `assets/` placeholders, `.github/workflows/` for GitHub Pages.
    *   **Milestones**: Docusaurus site builds locally; chapter stubs are accessible via sidebar.
    *   **Risk Factors**: Docusaurus version compatibility issues, initial GitHub Pages setup complexities.
    *   **Buffers**: 2 days.

2.  **Phase 2: RAG Backend Integration (Weeks 3-4)**
    *   **Deliverables**: Functional FastAPI RAG API skeleton (search endpoint), Qdrant vector database integrated, Neon (PostgreSQL) schema for metadata, content ingestion pipeline (docs → embeddings → Qdrant/Neon).
    *   **Milestones**: RAG search API returns relevant document chunks; basic ingestion process works.
    *   **Risk Factors**: Embedding model selection and performance, vector database tuning, data ingestion complexities.
    *   **Buffers**: 3 days.

3.  **Phase 3: Bonus Features (Weeks 5-6)**
    *   **Deliverables**: Better-Auth integration for user management, personalization logic (e.g., storing user progress/preferences), Urdu translation toggle (functional front-end component, backend placeholder for translation service).
    *   **Milestones**: User login/registration works; content personalized based on simple rules; Urdu toggle appears.
    *   **Risk Factors**: Auth security implementation, effective personalization algorithm design, potential API costs for external translation.
    *   **Buffers**: 3 days.

4.  **Phase 4: Testing, Validation, & Deployment (Week 6, overlapping)**
    *   **Deliverables**: Comprehensive testing of all integrated features, final quality validation against defined metrics, successful deployment to GitHub Pages, final review of `constitution.md`.
    *   **Milestones**: All quality validation targets met; textbook deployed and accessible online.
    *   **Risk Factors**: Integration bugs, deployment pipeline failures, last-minute content review changes.
    *   **Buffers**: 2 days.

## Dependencies

-   **Docusaurus Site Readiness**: The Docusaurus v3 static site (Phase 1) must be fully set up and buildable *before* significant chapter content writing begins.
-   **RAG Database Preparedness**: Qdrant and Neon databases (Phase 2) must be provisioned and configured *before* the embedding and chatbot features can be fully developed or content ingested.
-   **Bonus Features Sequential Implementation**: Auth (Better-Auth) should precede Personalization, as personalization often relies on user identity. Urdu Translation can be developed somewhat independently but will integrate with the Docusaurus frontend.

## Decisions Requiring Documentation

-   **Qdrant vs. in-memory vector DB**: Discuss tradeoffs between scalability, persistence, performance, and operational overhead. (Decision: Qdrant chosen for scalability and production readiness, even for initial skeleton).
-   **Research approach: concurrent vs. upfront**: Document rationale for concurrent gathering/writing, considering the iterative nature of academic content creation. (Decision: Concurrent for agility).
-   **Personalization depth**: Clarify scope—e.g., simple tips/resource recommendations vs. dynamic content generation/full chapter rewrites. (Decision: Simple tips/resource recommendations for MVP).
-   **Simulation strategy**: Explore tradeoffs between purely cloud-based simulations vs. hybrid approaches leveraging edge robotics kits. (Decision: Entirely simulation-focused, leveraging cloud resources or local high-performance workstations, no direct edge kit integration required for textbook content).
-   **Accessibility, UX, Computational Tradeoffs**: Document specific decisions made regarding WCAG compliance, user experience choices impacting performance, and computational resources allocated for RAG/embeddings.

## Technical Requirements

-   **MIT License**: The entire project codebase and textbook content will be released under the MIT License, promoting open access and reuse.
-   **WCAG-accessible design**: The Docusaurus frontend must adhere to WCAG 2.1 AA guidelines, ensuring accessibility for all users.
-   **Prefer free-tier services**: Where cloud services are utilized (e.g., for Neon PostgreSQL, or potential managed Qdrant), a strong preference for free-tier options will guide architectural choices to minimize costs.
-   **Entirely simulation-focused**: The textbook content, examples, and capstone project will strictly use simulation environments (Gazebo/Unity, NVIDIA Isaac), requiring no physical hardware for readers to follow along.
-   **APA style citations**: All academic references within the textbook content must strictly follow APA 7th edition citation style.

## Timeline

**Project Duration**: 6 Weeks

*   **Week 1**:
    *   **Phase 1: Core Textbook Skeleton (Part 1)**
        *   Setup Docusaurus v3 project (local buildable)
        *   Define initial chapter structure & empty MDX stubs (Intro, ROS 2 Module 1)
    *   **Milestones**: Docusaurus site serves local content; ROS 2 chapters accessible.
    *   **Risks**: Docusaurus config complexities.

*   **Week 2**:
    *   **Phase 1: Core Textbook Skeleton (Part 2)**
        *   Complete chapter stubs (Digital Twin, NVIDIA Isaac, VLA, Capstone, Conclusion)
        *   Setup `specs/` directory, `assets/` placeholders, `.github/workflows/` for GitHub Pages.
    *   **Milestones**: All chapter stubs are present and navigable; basic GitHub Actions workflow set up.
    *   **Risks**: GitHub Actions YAML errors.

*   **Week 3**:
    *   **Phase 2: RAG Backend Integration (Part 1)**
        *   FastAPI project setup & basic search endpoint skeleton
        *   Neon (PostgreSQL) database provisioning & schema definition for metadata
    *   **Milestones**: FastAPI app runs locally; database connection established.
    *   **Risks**: Database connection issues, FastAPI routing setup.

*   **Week 4**:
    *   **Phase 2: RAG Backend Integration (Part 2)**
        *   Qdrant vector database integration & basic configuration
        *   Content ingestion pipeline (docs → embeddings → Qdrant/Neon)
    *   **Milestones**: RAG search returns basic results; content can be embedded and stored.
    *   **Risks**: Embedding model choice, Qdrant cluster setup.

*   **Week 5**:
    *   **Phase 3: Bonus Features**
        *   Better-Auth integration (user login/registration placeholders)
        *   Personalization logic (e.g., storing user progress in Neon)
        *   Urdu translation toggle (frontend component, backend placeholder)
    *   **Milestones**: Auth endpoints functional; personalization data stored; translation button visible.
    *   **Risks**: Auth security, UX for personalization.

*   **Week 6**:
    *   **Phase 4: Testing, Validation, & Deployment**
        *   Implement RAG benchmark queries & achieve ≥90% accuracy
        *   Conduct full user flow tests (signup → personalized → Urdu → quiz)
        *   Deploy Docusaurus site to GitHub Pages
        *   Final review and adjustments to constitution.
    *   **Milestones**: All quality metrics met; site live on GitHub Pages.
    *   **Risks**: Deployment failures, last-minute bugs.

**Risk Factors & Buffers**:
-   **Technical Complexity**: Integrating Docusaurus, FastAPI, Qdrant, Neon can present unforeseen challenges; a 5-day buffer is allocated across phases.
-   **Scope Creep**: Maintaining strict adherence to the defined feature set and avoiding unnecessary additions.
-   **External Dependencies**: Reliance on external services (e.g., embedding models, free-tier limitations) might introduce instability or cost implications.
-   **Content Generation Delays**: While this plan is for the skeleton, delays in actual chapter content writing can impact overall project completion.