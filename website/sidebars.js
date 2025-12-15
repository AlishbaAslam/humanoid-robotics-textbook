// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['physical-ai-intro/introduction'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 (Weeks 3-5)',
      items: [
        'module-1-ros2/intro',  // Module 1: The Robotic Nervous System (ROS 2)
        'module-1-ros2/chapter-1-ros2/index',  // Chapter 1: Fundamentals of ROS 2 Nodes, Topics, and Services
        'module-1-ros2/chapter-2-python-agents/index',  // Chapter 2: Integrating Python Agents with ROS 2 via rclpy
        'module-1-ros2/chapter-3-urdf-modeling/index',  // Chapter 3: Modeling Humanoid Robots with URDF
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Weeks 6-7)',
      items: [
        'module-2-digital-twin/intro',  // Module 2: The Digital Twin (Gazebo & Unity)
        'module-2-digital-twin/chapter-1-gazebo/index',  // Chapter 1: Physics Simulation and Environment Building in Gazebo
        'module-2-digital-twin/chapter-2-unity/index',  // Chapter 2: High-Fidelity Rendering and Human-Robot Interaction in Unity
        'module-2-digital-twin/chapter-3-sensors/index',  // Chapter 3: Sensor Simulation (LiDAR, Depth Cameras, IMUs) and Integration
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac (Weeks 8-10)',
      items: [
        'module-3-ai-robot-brain/intro',  // Module 3: AI Robot Brain (NVIDIA Isaac)
        'module-3-ai-robot-brain/chapter-1-isaac-sim/index',  // Chapter 1: Isaac Sim - Physics Simulation and Environment Building
        'module-3-ai-robot-brain/chapter-2-isaac-ros/index',  // Chapter 2: Isaac ROS - Perception and Navigation Pipelines
        'module-3-ai-robot-brain/chapter-3-nav2-humanoid/index',  // Chapter 3: Navigation 2 (Nav2) for Humanoid Robots
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Educational Module (Weeks 12-13)',
      items: [
        'module-4-vla-educational-module/intro',  // Module 4: Voice, Language, and Action (VLA) Educational Module
        'module-4-vla-educational-module/chapter-1-voice-to-action/index',  // Chapter 1: Voice Command Processing and Action Mapping
        'module-4-vla-educational-module/chapter-2-cognitive-planning/index',  // Chapter 2: Cognitive Planning and LLM Integration
        'module-4-vla-educational-module/chapter-3-capstone-project/index',  // Chapter 3: Capstone Project - Integrating VLA in Humanoid Robotics
      ],
    },
  ],
};

export default sidebars;