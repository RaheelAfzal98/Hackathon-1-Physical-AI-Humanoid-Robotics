# Chapter Model

## Properties
- **ID**: Unique identifier for the chapter
- **Title**: Chapter title
- **Content**: Markdown content of the chapter
- **Module**: Module that contains the chapter (e.g. "Module 1: ROS 2 Nervous System")
- **Dependencies**: Other chapters/modules this chapter depends on
- **Learning Objectives**: List of learning objectives for the chapter
- **Code Examples**: Embedded or referenced code examples
- **Assets**: Images, diagrams, and other assets

## Example
```yaml
ID: "ch1"
Title: "Introduction to ROS 2"
Content: "This chapter introduces the core concepts of ROS 2..."
Module: "Module 1: ROS 2 Nervous System"
Dependencies: []
Learning Objectives:
  - Understand what ROS 2 is and its role in robotics
  - Learn about Nodes, Topics, and Services
  - Be able to create a simple ROS 2 node
Code Examples:
  - Simple publisher-subscriber example
Assets:
  - ros2-architecture.png
  - node-topic-service-diagram.png
```