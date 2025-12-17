# Quickstart: Textbook Repository Skeleton

## Overview
This document provides a quick guide to setting up and running the Physical AI & Humanoid Robotics Textbook project after the repository skeleton has been generated.

## 1. Prerequisites
-   Node.js (LTS version, e.g., 18.x or newer)
-   npm or Yarn (package manager)
-   Python 3.10+
-   pip (Python package installer)
-   Git
-   Docker (for RAG backend services)

## 2. Setting Up the Docusaurus Frontend

1.  **Navigate to the project root:**
    ```bash
    cd physical-ai-humanoid-robotics-textbook
    ```
2.  **Install Docusaurus dependencies:**
    ```bash
    npm install # or yarn install
    ```
3.  **Start the Docusaurus development server:**
    ```bash
    npm run start # or yarn start
    ```
    This will open the textbook in your browser at `http://localhost:3000`.

## 3. Setting Up the RAG Backend (Docker required)

1.  **Ensure Docker is running.**
2.  **Navigate to the RAG directory:**
    ```bash
    cd physical-ai-humanoid-robotics-textbook/rag
    ```
3.  **Build and run the Docker containers:**
    ```bash
    docker-compose up --build
    ```
    This will start the FastAPI application, Qdrant, and Neon (PostgreSQL) services.
    The FastAPI RAG API should be accessible at `http://localhost:8000`.

## 4. GitHub Pages Deployment

The project includes a GitHub Actions workflow (`.github/workflows/pages-deploy.yml`) for automatic deployment to GitHub Pages.
1.  Push your changes to the `main` branch.
2.  Ensure GitHub Pages is configured to deploy from the `gh-pages` branch (or `main` branch with `/docs` folder, depending on your `docusaurus.config.js`).
3.  Monitor the GitHub Actions tab in your repository for deployment status.

## 5. Next Steps
-   Start writing content in the `chapters/` directory.
-   Explore and develop the RAG backend's functionalities.
-   Implement the bonus features (Auth, Personalization, Urdu Translation).
