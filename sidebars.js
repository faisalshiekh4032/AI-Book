/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Course Overview',
      items: [
        'overview/about',
        'overview/objectives',
        'overview/schedule',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        'module1/introduction',
        'module1/architecture',
        'module1/nodes-topics',
        'module1/python-rclpy',
        'module1/urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin & Simulation',
      items: [
        'module2/overview',
        'module2/gazebo',
        'module2/unity',
        'module2/sensors',
        'module2/physics',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      items: [
        'module3/introduction',
        'module3/isaac-sim',
        'module3/isaac-ros',
        'module3/vslam',
        'module3/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module4/overview',
        'module4/voice-control',
        'module4/llm-integration',
        'module4/multimodal',
        'module4/capstone',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Setup',
      items: [
        'hardware/requirements',
        'hardware/workstation',
        'hardware/edge-computing',
        'hardware/robots',
      ],
    },
    {
      type: 'category',
      label: 'Assessments & Projects',
      items: [
        'assessments/guidelines',
        'assessments/projects',
        'assessments/evaluation',
      ],
    },
  ],
};

module.exports = sidebars;
