# Physical AI & Humanoid Robotics Textbook

This repository contains a comprehensive textbook on Physical AI and Humanoid Robotics, created using the Spec-Kit Plus methodology with AI assistance.

## Overview

This textbook teaches students and developers how to build humanoid robots using modern robotics technologies:

- **ROS 2**: As the communication nervous system
- **Gazebo & Unity**: For digital twin simulation
- **NVIDIA Isaac™**: For AI-driven robot brain
- **Vision-Language-Action (VLA)**: For natural human-robot interaction

## Book Structure

The textbook consists of four core modules:

### Module 1: ROS 2 Nervous System
Learn how ROS 2 serves as the communication nervous system of robots, covering Nodes, Topics, Services, and practical implementation using rclpy.

### Module 2: The Digital Twin (Gazebo & Unity)
Explore Digital Twin concepts using Gazebo and Unity, including physics simulation, environment creation, collision systems, and sensor simulation (LiDAR, depth cameras, IMUs).

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
Discover how NVIDIA Isaac serves as the AI-driven brain of humanoid robots, covering Isaac Sim, synthetic data generation, Isaac ROS acceleration, VSLAM, and Nav2 path planning for bipedal movement.

### Module 4: Vision-Language-Action (VLA)
Understand how LLMs, perception, and robotics converge into full autonomous humanoid behavior, with Voice-to-Action pipelines and LLM-based cognitive planning.

## Technology Stack

- **Framework**: Docusaurus for textbook documentation
- **Robotics**: ROS 2 Humble Hawksbill
- **Simulation**: Gazebo and Unity
- **AI/ML**: NVIDIA Isaac, OpenAI Whisper, GPT models
- **Backend**: FastAPI for RAG system
- **Vector Storage**: Qdrant for knowledge retrieval

## Installation

### Prerequisites
- Node.js (for Docusaurus)
- ROS 2 Humble Hawksbill
- Python 3.10+
- NVIDIA Isaac (optional, for Module 3)

### Setup Docusaurus Development Environment

```bash
# Clone the repository
git clone <repository-url>
cd book

# Install dependencies
npm install

# Start the development server
npm start
```

The textbook will be available at `http://localhost:3000`.

## Content Philosophy

This textbook follows the Digital Twin concept where virtual representations of the learning process mirror the actual development of humanoid robots. Students can learn, experiment, and validate concepts in a safe virtual environment before applying them to physical robots.

## Development Approach

This textbook was created using:
- **Spec-Kit Plus methodology**: Systematic approach to specification-driven development
- **AI assistance**: Claude Code for content generation and refinement
- **Modular design**: Each module builds upon previous concepts while remaining independent
- **Reproducible examples**: All code examples are tested and validated

## Contributing

This project welcomes contributions! Please submit pull requests with improvements to content, examples, or documentation.

## License

This textbook is provided under the terms specified in the project constitution. All content is original and created with attention to zero-plagiarism principles.

## Acknowledgments

This textbook was created using the innovative Spec-Kit Plus methodology combined with AI assistance, demonstrating the power of human-AI collaboration in educational content creation.