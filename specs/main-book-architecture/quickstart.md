# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This quickstart guide will help you get started with the Physical AI & Humanoid Robotics textbook, including both the content and the RAG-powered learning tools.

## Prerequisites
- A modern web browser to access the textbook
- For hands-on exercises:
  - ROS 2 Humble Hawksbill installed
  - Python 3.10+
  - Access to OpenAI API for some advanced exercises (optional)

## Accessing the Textbook

### Online Access
The textbook is available at [your-deployment-url] where you can browse all modules and chapters. Use the left sidebar to navigate between modules:

1. **Module 1: ROS 2 Nervous System** - Covers fundamental ROS 2 concepts
2. **Module 2: The Digital Twin (G&A Unity)** - Covers simulation concepts
3. **Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - Covers AI-driven robotics
4. **Module 4: Vision-Language-Action (VLA)** - Covers advanced human-robot interaction

### Local Development
To run the textbook locally:

```bash
# Clone the repository
git clone <your-repo-url>
cd book

# Install dependencies
npm install

# Start the development server
npm start
```

The textbook will be available at http://localhost:3000

## Using the RAG-Powered Chatbot

The textbook includes an intelligent chatbot powered by Retrieval-Augmented Generation (RAG) that can answer questions about the content:

1. Ask questions in the chat interface
2. The system will retrieve relevant textbook passages
3. It will generate answers grounded in the textbook content
4. You can verify the sources of the information provided

### Example Queries
- "Explain ROS 2 pubsub communication pattern"
- "How do I create a URDF for a humanoid robot?"
- "What are the components of the VLA system?"
- "How does Isaac Sim help with perception?"

## Hands-On Exercises

Each chapter includes conceptual explanations and code examples. For modules involving ROS 2:

1. Ensure ROS 2 Humble Hawksbill is sourced: `source /opt/ros/humble/setup.bash`
2. Create a new workspace: `mkdir -p ~/textbook_ws/src && cd ~/textbook_ws`
3. Follow the examples in each chapter, making sure to understand the concepts before running code

## Module-Specific Quickstarts

### Module 1: ROS 2 Nervous System
Start with understanding nodes, topics, and services. Practice creating simple publisher and subscriber nodes.

### Module 2: The Digital Twin (Gazebo & Unity)
Focus on simulation concepts. Learn to create simple environments in both Gazebo and Unity.

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
Work with Isaac Sim to understand synthetic data generation and navigation systems.

### Module 4: Vision-Language-Action (VLA)
Combine all previous modules to create an intelligent robot that responds to voice commands.

## Troubleshooting

### Textbook Navigation
- If pages don't load, refresh the browser
- Use the search function to quickly find specific topics
- Check the table of contents in each module for an overview

### RAG System
- If the chatbot doesn't respond, check your internet connection
- Very specific questions might require broader search terms
- If results seem incorrect, try rephrasing the question

### Code Examples
- Ensure your ROS 2 environment is properly sourced
- Check that all required packages are installed
- Refer to the official ROS 2 documentation for additional troubleshooting

## Next Steps

1. Begin with Module 1 if you're new to ROS 2
2. Work through each chapter sequentially within a module
3. Complete the exercises and examples
4. Use the RAG chatbot to clarify any concepts you find challenging
5. Progress to the next module only after understanding the current one

## Support and Community

- For content questions, use the RAG chatbot
- For technical issues with examples, refer to the official documentation for each technology
- For textbook-specific questions, check the GitHub repository for issues or create a new one