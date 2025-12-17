# Physical AI & Humanoid Robotics Textbook

This repository hosts the "Physical AI & Humanoid Robotics" open-source textbook, built with Docusaurus v3. It covers foundational concepts in ROS 2, digital twins, NVIDIA Isaac, Vision-Language-Action (VLA) pipelines, and culminates in an end-to-end autonomous humanoid capstone project.

The project also includes a FastAPI backend for Retrieval-Augmented Generation (RAG) capabilities and bonus features like user authentication, personalization, and Urdu translation.

## Project Structure

-   `textbook-site/`: The Docusaurus frontend for the textbook.
-   `rag-backend/`: FastAPI backend for RAG functionalities and bonus features.
-   `bonus-features/`: Contains specific implementations for authentication, personalization, and translation.
-   `.github/workflows/`: GitHub Actions for CI/CD, including deployment to GitHub Pages.

## Setup Instructions

### Prerequisites

Ensure you have the following installed:
-   Node.js (LTS version, >=18.0)
-   npm (comes with Node.js)
-   Python 3.9+
-   pip (comes with Python)
-   (Optional but Recommended) A Python virtual environment tool (e.g., `venv`, `conda`)

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/humanoid-textbook.git
cd humanoid-textbook
```

### 2. Frontend Setup (Docusaurus)

Navigate to the `textbook-site` directory and install dependencies:

```bash
cd textbook-site
npm install
```

To run the Docusaurus development server:

```bash
npm run start
```

This will open the site in your browser at `http://localhost:3000`.

To build the static site:

```bash
npm run build
```

The built site will be located in the `build/` directory.

### 3. Backend Setup (FastAPI RAG)

Navigate back to the project root and then into the `rag-backend` directory:

```bash
cd .. # if you are in textbook-site
cd rag-backend
```

Create and activate a Python virtual environment:

```bash
python -m venv venv
# On Windows:
.\venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate
```

Install Python dependencies:

```bash
pip install -r requirements.txt # You might need to create this file
```
**Note**: The `requirements.txt` file is not yet created, but would contain `fastapi`, `uvicorn`, `sqlalchemy`, `psycopg2-binary`, `qdrant-client`, `passlib[bcrypt]`, `python-markdown`, `beautifulsoup4`, etc.

Run the backend development server:

```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

You can test the backend health check by visiting `http://localhost:8000/health` or `http://localhost:8000/docs` for the OpenAPI documentation.

To run the provided backend test script (Windows PowerShell example):

```powershell
.\run_backend.ps1
```

### 4. Database & Vector Store Configuration

The backend requires a PostgreSQL database (e.g., Neon) and a Qdrant vector store.
-   **PostgreSQL**: Update the `DATABASE_URL` in `rag-backend/database.py` (or set as environment variable) to point to your PostgreSQL instance.
-   **Qdrant**: Ensure a Qdrant instance is running (e.g., via Docker) and update `QDRANT_HOST`, `QDRANT_PORT` in `rag-backend/vector_db.py` (or set as environment variables).

### 5. Ingest Textbook Content

To ingest the textbook content into the RAG system, run the ingestion script from the `rag-backend` directory after the database and Qdrant are configured:

```bash
# Make sure your virtual environment is active
python -m ingestion.ingest
```

## Deployment to GitHub Pages

This repository is configured for automatic deployment to GitHub Pages via GitHub Actions.

1.  **Ensure `textbook-site/docusaurus.config.ts` has `baseUrl: '/'` and `organizationName`, `projectName` are correctly set.**
2.  **Push your changes to the `main` branch.** The GitHub Actions workflow (`.github/workflows/deploy.yml`) will automatically build the Docusaurus site and deploy it to the `gh-pages` branch.
3.  **Configure GitHub Pages**: In your repository settings on GitHub, navigate to "Pages" and select the `gh-pages` branch as your source, with `/ (root)` as the folder.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact

For questions or feedback, please contact [rajlaiba65@mail.com].
