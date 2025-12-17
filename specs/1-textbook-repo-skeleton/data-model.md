# Data Model: Textbook Repository Skeleton

## Overview
This document outlines the conceptual data model for the Physical AI & Humanoid Robotics Textbook project, focusing on the RAG backend, user personalization, and content structure necessary for the initial skeleton and future enhancements.

## 1. Textbook Content (for RAG)

### 1.1. Module
Represents a major thematic module within the textbook (e.g., ROS 2, Digital Twin).
-   `module_id`: Unique identifier (e.g., "01-ros2")
-   `title`: Title of the module (e.g., "ROS 2 (Robotic Nervous System)")
-   `description`: Brief summary of the module.
-   `order`: Numerical order for display.

### 1.2. Chapter
Represents an individual chapter within a module.
-   `chapter_id`: Unique identifier (e.g., "01-ros2/chapter1")
-   `module_id`: Foreign key referencing the `Module` it belongs to.
-   `title`: Chapter title (e.g., "ROS 2 Basics").
-   `file_path`: Path to the Markdown/MDX file (e.g., `chapters/01-ros2/chapter1.md`).
-   `learning_objectives`: Array of strings.
-   `key_concepts`: Array of strings.
-   `order`: Numerical order within its module.

### 1.3. Content Segment (for Embeddings)
Represents a chunk of text from a chapter used for RAG.
-   `segment_id`: Unique identifier.
-   `chapter_id`: Foreign key referencing the `Chapter` it belongs to.
-   `text_content`: The raw text segment.
-   `embedding`: Vector representation of `text_content` (stored in Qdrant).
-   `context_metadata`: JSON blob containing section, subsection, page number, etc.
-   `source_url`: URL to the rendered content.

## 2. User (for Auth & Personalization)

### 2.1. User
Represents a user of the textbook platform.
-   `user_id`: Unique identifier (from Better-Auth).
-   `username`: User's display name.
-   `email`: User's email address.
-   `preferences`: JSON blob for personalization settings (e.g., theme, preferred learning style).
-   `progress`: JSON blob tracking chapter completion, quiz scores, etc.

## 3. Relationships

-   `Chapter` has a one-to-many relationship with `Content Segment`.
-   `Module` has a one-to-many relationship with `Chapter`.
-   `User` interactions will influence personalization logic and data stored (e.g., `progress`, `preferences`).

## 4. Storage Locations

-   **Qdrant**: Stores `Content Segment` embeddings and `segment_id`.
-   **Neon (PostgreSQL)**: Stores `Module`, `Chapter`, `Content Segment` (metadata only), and `User` data.
