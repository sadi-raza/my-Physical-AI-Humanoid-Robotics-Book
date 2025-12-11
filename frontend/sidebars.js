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
  // Manual sidebar structure for the Physical AI & Humanoid Robotics textbook
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'ROS2',
      items: [
        'ros2/intro',
        'ros2/chapter-1',
        'ros2/chapter-2',
        'ros2/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Gazebo/Unity',
      items: [
        'gazebo-unity/intro',
        'gazebo-unity/chapter-1',
        'gazebo-unity/chapter-2',
        'gazebo-unity/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Isaac',
      items: [
        'isaac/intro',
        'isaac/chapter-1',
        'isaac/chapter-2',
        'isaac/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'VLA',
      items: [
        'vla/intro',
        'vla/chapter-1',
        'vla/chapter-2',
      ],
    },
    {
      type: 'category',
      label: 'Capstone',
      items: [
        'capstone/intro',
        'capstone/chapter-1',
        'capstone/chapter-2',
      ],
    },
  ],
};

export default sidebars;
