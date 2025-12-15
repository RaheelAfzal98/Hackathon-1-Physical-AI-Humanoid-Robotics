module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro', 'ros2-setup'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-1-ros2/chapter-1-introduction',
        'module-1-ros2/chapter-2-nodes-topics-services',
        'module-1-ros2/chapter-3-rclpy-implementation',
        'module-1-ros2/chapter-4-urdf-humanoid',
        'module-1-ros2/chapter-5-complete-example'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/chapter-1-introduction',
        'module-2-digital-twin/chapter-2-physics-simulation',
        'module-2-digital-twin/chapter-3-gazebo-integration',
        'module-2-digital-twin/chapter-4-unity-visualization',
        'module-2-digital-twin/chapter-5-sensor-simulation',
        'module-2-digital-twin/chapter-6-complete-example'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-ai-robot-brain/chapter-1-introduction',
        'module-3-ai-robot-brain/chapter-2-isaac-sim',
        'module-3-ai-robot-brain/chapter-3-synthetic-data',
        'module-3-ai-robot-brain/chapter-4-isaac-ros-acceleration',
        'module-3-ai-robot-brain/chapter-5-nav2-path-planning',
        'module-3-ai-robot-brain/chapter-6-complete-example'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/chapter-1-introduction',
        'module-4-vla/chapter-2-voice-command-ingestion',
        'module-4-vla/chapter-3-cognitive-planning',
        'module-4-vla/chapter-4-complete-capstone-pipeline',
        'module-4-vla/chapter-5-integration-with-modules',
        'module-4-vla/chapter-6-validation-and-testing'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/glossary',
        'appendices/ros2-cheatsheet',
        'appendices/urdf-guide',
        'appendices/assessment-questions'
      ],
      collapsed: true,
    }
  ],
};