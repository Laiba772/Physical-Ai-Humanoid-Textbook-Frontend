---
id: 001
title: Create textbook repo skeleton specification
stage: spec
date: 2025-12-07
surface: agent
model: gemini-1.5-flash
feature: 1-textbook-repo-skeleton
branch: 1-textbook-repo-skeleton
user: user
command: /sp.specify physical AI & Humanoid Robotics Textbook (Repo Skeleton Specification) Task: Generate a complete empty repository structure (no chapter prose) for the Physical AI & Humanoid Robotics textbook using Docusaurus v3 + Spec-Kit Plus conventions and GitHub Pages deploy readiness. Target audience: Graduate CS students and AI researchers (Physical AI, ROS 2, NVIDIA Isaac, Gazebo/Unity, humanoid robotics). Output requirements (strict): Produce a single specification (markdown) that lists every required file and folder path (including empty placeholder files). After the spec, output a complete tree view in a tree -a style (folders first, then files). Do not write any chapter content—only empty markdown stubs (front-matter allowed). Keep naming consistent and Docusaurus-compatible (no ambiguous folder names). Repository requirements: Root folder: physical-ai-humanoid-robotics-textbook Docusaurus v3 structure (ready to build) including required config files (e.g., Docusaurus config + sidebars + package scripts for GitHub Pages). Spec-Kit Plus compatibility: include specs/ hierarchy expected by Spec-Kit (at minimum: specs/textbook/ with spec/plan/tasks placeholders). Required top-level folders (must exist): specs/ (Spec-Kit artifacts and project docs) chapters/ (textbook chapter markdown stubs) assets/ (images/diagrams/videos/links placeholders) rag/ (RAG backend skeleton: API + vector DB + ingestion layout) .github/workflows/ (GitHub Actions CI for Pages) Required top-level files (must exist): .gitignore README.md Docusaurus config (e.g., docusaurus.config.js or equivalent) Sidebars config (e.g., sidebars.js or equivalent) GitHub Pages deployment workflow YAML (placeholder runnable pipeline) Bonus feature placeholders (must be present as empty/seed files): Auth (Better-Auth integration placeholder) Personalization (per-user learning-routing placeholder) Urdu translation (chapter-level translation toggle placeholder) Module & chapter mapping (create empty chapter files with clear numbering): Module 1 — ROS 2 (Robotic Nervous System) chapters/01-ros2/ → stubs for core ROS 2 concepts (nodes/topics/services), rclpy bridge, URDF basics Module 2 — Digital Twin (Gazebo & Unity) chapters/02-digital-twin/ → stubs for physics simulation, sensors (LiDAR/depth/IMU), environment + visualization Module 3 — NVIDIA Isaac (AI-Robot Brain) chapters/03-isaac/ → stubs for perception, VSLAM/navigation, sim-to-real transfer Module 4 — Vision-Language-Action (VLA) chapters/04-vla/ → stubs for speech-to-action, LLM planning-to-ROS actions, multimodal interaction Capstone — Autonomous Humanoid chapters/05-capstone/ → end-to-end pipeline stub (voice → perception → navigation → manipulation) Assets (must include placeholder subfolders): assets/images/ (robot renderings / sensor diagrams) assets/diagrams/ (architecture + pipeline diagrams) assets/videos/ (simulation demo links placeholders) RAG backend layout (must include empty skeleton): FastAPI app entry file placeholder Neon (SQL) schema placeholder Qdrant (vector) configuration placeholder Ingestion pipeline folder (docs → embeddings) Final deliverable (what I expect you to print): Specification document (markdown) listing every required path (folders + files). Tree view (complete) of the generated structure. No extra narrative beyond the spec + tree.
labels: ["spec", "repository", "skeleton", "docusaurus"]
links:
  spec: specs/1-textbook-repo-skeleton/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/1-textbook-repo-skeleton/spec.md
- specs/1-textbook-repo-skeleton/plan.md
- specs/1-textbook-repo-skeleton/tasks.md
- specs/1-textbook-repo-skeleton/checklists/requirements.md
tests:
null
---

## Prompt

/sp.specify physical AI & Humanoid Robotics Textbook (Repo Skeleton Specification) Task: Generate a complete empty repository structure (no chapter prose) for the Physical AI & Humanoid Robotics textbook using Docusaurus v3 + Spec-Kit Plus conventions and GitHub Pages deploy readiness. Target audience: Graduate CS students and AI researchers (Physical AI, ROS 2, NVIDIA Isaac, Gazebo/Unity, humanoid robotics). Output requirements (strict): Produce a single specification (markdown) that lists every required file and folder path (including empty placeholder files). After the spec, output a complete tree view in a tree -a style (folders first, then files). Do not write any chapter content—only empty markdown stubs (front-matter allowed). Keep naming consistent and Docusaurus-compatible (no ambiguous folder names). Repository requirements: Root folder: physical-ai-humanoid-robotics-textbook Docusaurus v3 structure (ready to build) including required config files (e.g., Docusaurus config + sidebars + package scripts for GitHub Pages). Spec-Kit Plus compatibility: include specs/ hierarchy expected by Spec-Kit (at minimum: specs/textbook/ with spec/plan/tasks placeholders). Required top-level folders (must exist): specs/ (Spec-Kit artifacts and project docs) chapters/ (textbook chapter markdown stubs) assets/ (images/diagrams/videos/links placeholders) rag/ (RAG backend skeleton: API + vector DB + ingestion layout) .github/workflows/ (GitHub Actions CI for Pages) Required top-level files (must exist): .gitignore README.md Docusaurus config (e.g., docusaurus.config.js or equivalent) Sidebars config (e.g., sidebars.js or equivalent) GitHub Pages deployment workflow YAML (placeholder runnable pipeline) Bonus feature placeholders (must be present as empty/seed files): Auth (Better-Auth integration placeholder) Personalization (per-user learning-routing placeholder) Urdu translation (chapter-level translation toggle placeholder) Module & chapter mapping (create empty chapter files with clear numbering): Module 1 — ROS 2 (Robotic Nervous System) chapters/01-ros2/ → stubs for core ROS 2 concepts (nodes/topics/services), rclpy bridge, URDF basics Module 2 — Digital Twin (Gazebo & Unity) chapters/02-digital-twin/ → stubs for physics simulation, sensors (LiDAR/depth/IMU), environment + visualization Module 3 — NVIDIA Isaac (AI-Robot Brain) chapters/03-isaac/ → stubs for perception, VSLAM/navigation, sim-to-real transfer Module 4 — Vision-Language-Action (VLA) chapters/04-vla/ → stubs for speech-to-action, LLM planning-to-ROS actions, multimodal interaction Capstone — Autonomous Humanoid chapters/05-capstone/ → end-to-end pipeline stub (voice → perception → navigation → manipulation) Assets (must include placeholder subfolders): assets/images/ (robot renderings / sensor diagrams) assets/diagrams/ (architecture + pipeline diagrams) assets/videos/ (simulation demo links placeholders) RAG backend layout (must include empty skeleton): FastAPI app entry file placeholder Neon (SQL) schema placeholder Qdrant (vector) configuration placeholder Ingestion pipeline folder (docs → embeddings) Final deliverable (what I expect you to print): Specification document (markdown) listing every required path (folders + files). Tree view (complete) of the generated structure. No extra narrative beyond the spec + tree.

## Response snapshot

Successfully created feature specification for 'textbook-repo-skeleton' (branch '1-textbook-repo-skeleton'), including `spec.md`, `plan.md`, `tasks.md`, and `checklists/requirements.md`. All specification quality checks passed.

## Outcome

- ✅ Impact: Feature specification created and validated.
- 🧪 Tests: No automated tests run for spec generation, but internal quality checks passed.
- 📁 Files: Created `specs/1-textbook-repo-skeleton/spec.md`, `plan.md`, `tasks.md`, and `checklists/requirements.md`.
- 🔁 Next prompts: User can now proceed with `/sp.plan`.
- 🧠 Reflection: Manual creation of feature directory and files due to `create-new-feature.ps1` script parsing issues with long feature description argument. Successfully manually implemented the feature creation flow.

## Evaluation notes (flywheel)

- Failure modes observed: Initial attempt to run `create-new-feature.ps1` with JSON argument failed due to parsing issues. Second attempt without JSON failed due to script usage requirements. Manual creation succeeded.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate safe ways to pass long, complex strings as arguments to `run_shell_command` or modify `create-new-feature.ps1` to read description from a temporary file.
