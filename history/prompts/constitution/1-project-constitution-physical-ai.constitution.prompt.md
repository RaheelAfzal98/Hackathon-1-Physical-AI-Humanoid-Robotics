---
id: 1
title: project-constitution-physical-ai
stage: constitution
date: 2025-12-11
surface: agent
model: claude-sonnet-4
feature: none
branch: master
user: Hp
command: "/sp.constitution"
labels: ["constitution","specification","ai","robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:/sp
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
/sp.constitution Project: Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics

Core principles:
- Specification-driven development using Spec-Kit Plus and Claude Code
- Accuracy and correctness across robotics, AI systems, simulation, and humanoid control
- Consistency across all chapters and chatbot-generated responses
- Reproducibility of instructions, workflows, and code samples
- Clarity for an intermediate-to-advanced AI/robotics audience

Key standards:
- All robotics, simulation, and AI concepts must be verified with primary documentation 
  (ROS 2 docs, NVIDIA Isaac, Gazebo, Unity, OpenAI SDKs, FastAPI, Postgres, Qdrant)
- Writing style: clear, structured, Docusaurus-friendly
- All code must be runnable and follow best practices in ROS 2, rclpy, URDF, Gazebo/Unity, Isaac, and RAG stack
- All workflow diagrams must follow Spec-Kit Plus architecture formats
- RAG chatbot responses must be grounded strictly in book content or user-selected text
- Zero plagiarism; external claims must be cited in markdown-compatible references

Constraints:
- Book length: **18–20 chapters**, covering the full Physical AI & Humanoid Robotics curriculum including:
  - Foundations of Physical AI and Embodied Intelligence
  - Detailed multi-chapter coverage for:
      - Module 1 (ROS 2)
      - Module 2 (Gazebo & Unity)
      - Module 3 (NVIDIA Isaac)
      - Module 4 (Vision-Language-Action)
  - Additional chapters for:
      - Tools and Software Setup
      - System Design
      - Robotics Math
      - Control Theory Basics
      - Deployment
      - Testing & Validation
      - RAG Chatbot Engineering
      - Capstone: Autonomous Humanoid Robot Pipeline
- Must be implemented as a Docusaurus project deployed on GitHub Pages
- Must include a complete embedded RAG chatbot using:
  - OpenAI Agents/ChatKit
  - FastAPI backend
  - Neon Serverless Postgres
  - Qdrant Cloud Free Tier for vector search
- All required Spec-Kit Plus specification files must be generated and pass validation

Success criteria:
- Completed and polished book containing **18–20 coherent, technically accurate chapters**
- Fully deployed Docusaurus site on GitHub Pages
- Fully functional RAG chatbot with reliable grounding and correct retrieval
- All robotics pipelines reproducible (ROS → Gazebo/Unity → Isaac → VLA → Capstone)
- Capstone chapter describes the complete humanoid pipeline:
  - Voice → LLM planning → ROS 2 execution → navigation → perception → manipulation
- All Spec-Kit Plus validation and consistency checks pass without errors
```

## Response snapshot

I've successfully updated the project constitution for the "Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics" project. Here's my final summary:

## Summary

- **New Version**: 1.0.0 (initial version)
- **Bump Rationale**: This is the initial creation of the project constitution, establishing the foundational principles for developing the Physical AI & Humanoid Robotics textbook and RAG chatbot.

## Constitution Overview

The constitution establishes six core principles:
1. Specification-driven development using Spec-Kit Plus
2. Accuracy and correctness with primary documentation verification
3. Consistency across all chapters and chatbot responses
4. Reproducibility of all instructions, workflows, and code samples
5. Clarity for an intermediate-to-advanced AI/robotics audience
6. Zero-plagiarism commitment with proper citations

The document also defines technical standards (ROS 2, NVIDIA Isaac, Gazebo/Unity, RAG stack), deployment requirements (Docusaurus on GitHub Pages), and success criteria (18-20 chapters, functional RAG chatbot, capstone pipeline).

## Files Updated

- `.specify/memory/constitution.md` - Contains the complete project constitution

## Follow-up Items

- The ratification date is currently set to a placeholder (2025-06-13) and should be updated to the actual project start date if known

## Suggested Commit Message

`docs: create project constitution for Physical AI & Humanoid Robotics textbook`

The constitution is now ready for use and aligns with your project requirements for creating a comprehensive textbook with an integrated RAG chatbot on Physical AI and Humanoid Robotics.

## Outcome

- ✅ Impact: Created project constitution with 6 core principles and governance structure
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific details
- 🔁 Next prompts: Could proceed with /sp.specify to create feature specs
- 🧠 Reflection: Constitution properly captures the technical requirements and constraints from the user input

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): not applicable
- Prompt variant (if applicable): not applicable
- Next experiment (smallest change to try): begin creating feature specifications based on this constitution
- Prompt variant (if applicable): /sp.specify to create the first feature spec