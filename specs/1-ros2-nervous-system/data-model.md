# Data Model: ROS 2 Nervous System Module

## Chapter Entity
- ID: Unique identifier for the chapter
- Title: Chapter title
- Content: Markdown content of the chapter
- Module: Module that contains the chapter (e.g. "Module 1: ROS 2 Nervous System")
- Dependencies: Other chapters/modules this chapter depends on
- Learning Objectives: List of learning objectives for the chapter
- Code Examples: Embedded or referenced code examples
- Assets: Images, diagrams, and other assets

## Module Entity
- ID: Unique identifier for the module
- Title: Module title
- Chapters: List of chapters in the module
- Learning Outcomes: List of outcomes students should achieve
- Prerequisites: Prerequisites for the module
- Difficulty: Difficulty level

## Book Entity
- Modules: List of all modules in the book
- Navigation: Structure for navigation and table of contents
- Metadata: Information about the book (title, authors, version)
- Integration Points: Where RAG system connects to book content

## Humanoid Robot Model Entity
- ID: Unique identifier for the robot model
- URDF: URDF content for the robot
- Joints: List of joints in the robot
- Links: List of links in the robot
- Physical properties: Mass, inertia, and other physical properties
- Description: Human-readable description of the robot

## Code Example Entity
- ID: Unique identifier for the code example
- Language: Programming language (e.g., Python, C++)
- Content: The actual code content
- Associated Chapter: Chapter that contains this code example
- Test Status: Whether the code example has been tested and validated
- Dependencies: Other code or ROS packages required to run this example