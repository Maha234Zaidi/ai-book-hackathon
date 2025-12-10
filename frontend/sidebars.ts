import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Robotic Communication Systems (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1-ros2-nervous-system/index',
        'module-1-ros2-nervous-system/chapter-1-ros2-intro',
        'module-1-ros2-nervous-system/chapter-2-nodes-topics-services',
        'module-1-ros2-nervous-system/chapter-3-ai-agents-control',
        'module-1-ros2-nervous-system/chapter-4-urdf-modeling',
        'module-1-ros2-nervous-system/chapter-5-mini-project',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Simulation (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'module2-digital-twin/index',
        {
          type: 'category',
          label: 'Introduction to Digital Twin Concepts',
          collapsed: true,
          items: [
            'module2-digital-twin/introduction-digital-twin-concepts',
          ],
        },
        {
          type: 'category',
          label: 'Physics Simulation in Gazebo',
          collapsed: true,
          items: [
            'module2-digital-twin/physics-simulation-gazebo',
            'module2-digital-twin/physics-engine-configuration',
            'module2-digital-twin/gravity-examples',
            'module2-digital-twin/collision-detection-examples',
            'module2-digital-twin/urdf-physical-properties-examples',
            'module2-digital-twin/physics-troubleshooting',
          ],
        },
        {
          type: 'category',
          label: 'High-Fidelity Rendering in Unity',
          collapsed: true,
          items: [
            'module2-digital-twin/rendering-unity',
            'module2-digital-twin/high-fidelity-rendering-unity',
            'module2-digital-twin/lighting-setup-unity',
            'module2-digital-twin/texture-application-unity',
            'module2-digital-twin/model-import-consistency',
          ],
        },
        {
          type: 'category',
          label: 'Sensor Simulation',
          collapsed: true,
          items: [
            'module2-digital-twin/sensor-simulation',
            'module2-digital-twin/sensor-simulation-details',
            'module2-digital-twin/gazebo-sensor-noise-models',
            'module2-digital-twin/sensor-calibration-techniques',
            'module2-digital-twin/validation-tests-sensor-examples',
          ],
        },
        {
          type: 'category',
          label: 'Integration & Best Practices',
          collapsed: true,
          items: [
            'module2-digital-twin/integration-best-practices',
            'module2-digital-twin/integration-best-practices-reproducibility',
            'module2-digital-twin/optimization-tips-unity-rendering',
            'module2-digital-twin/digital-twin-architecture',
            'module2-digital-twin/coordinate-systems',
          ],
        },
        {
          type: 'category',
          label: 'Troubleshooting',
          collapsed: true,
          items: [
            'module2-digital-twin/troubleshooting-gazebo-unity-integration',
          ],
        },
        {
          type: 'category',
          label: 'Module Summary',
          collapsed: true,
          items: [
            'module2-digital-twin/summary-key-takeaways',
          ],
        },
        {
          type: 'category',
          label: 'Module Handoff',
          collapsed: true,
          items: [
            'module2-digital-twin/module-handoff-checklist',
          ],
        },
        {
          type: 'category',
          label: 'Hands-on Exercises',
          collapsed: true,
          items: [
            'module2-digital-twin/hands-on-exercise-1',
            'module2-digital-twin/hands-on-exercise-2',
            'module2-digital-twin/sensor-data-comparison-exercise',
            'module2-digital-twin/hands-on-exercise-4',
            'module2-digital-twin/hands-on-exercise-5',
          ],
        },
        {
          type: 'category',
          label: 'Setup and Configuration',
          collapsed: true,
          items: [
            'module2-digital-twin/environment-setup',
            'module2-digital-twin/ros2-workspace-setup',
            'module2-digital-twin/unity-scene-setup',
            'module2-digital-twin/ros2-communication',
          ],
        },
        {
          type: 'category',
          label: 'Validation and Testing',
          collapsed: true,
          items: [
            'module2-digital-twin/validation-scripts',
            'module2-digital-twin/physics-validation-tests',
            'module2-digital-twin/visual-diagrams',
            'module2-digital-twin/citations-references',
            'module2-digital-twin/official-documentation-citations',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'AI-Robot Brain (NVIDIA Isaac™)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3-ai-brain/index',
        {
          type: 'category',
          label: 'Introduction to NVIDIA Isaac',
          collapsed: true,
          items: [
            'module-3-ai-brain/introduction-to-isaac/index',
            'module-3-ai-brain/introduction-to-isaac/isaac-sim-basics',
            'module-3-ai-brain/introduction-to-isaac/isaac-ros-basics',
            'module-3-ai-brain/introduction-to-isaac/applications-examples',
            'module-3-ai-brain/introduction-to-isaac/exercise-1',
            'module-3-ai-brain/introduction-to-isaac/examples',
          ],
        },
        {
          type: 'category',
          label: 'Photorealistic Simulation',
          collapsed: true,
          items: [
            'module-3-ai-brain/photorealistic-simulation/index',
            'module-3-ai-brain/photorealistic-simulation/synthetic-data-generation',
            'module-3-ai-brain/photorealistic-simulation/sensor-integration',
            'module-3-ai-brain/photorealistic-simulation/omniverse-rendering',
            'module-3-ai-brain/photorealistic-simulation/exercise-2',
            'module-3-ai-brain/photorealistic-simulation/examples',
          ],
        },
        {
          type: 'category',
          label: 'VSLAM and Navigation',
          collapsed: true,
          items: [
            'module-3-ai-brain/vslam-navigation/index',
            'module-3-ai-brain/vslam-navigation/vslam-theory',
            'module-3-ai-brain/vslam-navigation/hardware-acceleration',
            'module-3-ai-brain/vslam-navigation/nav-examples',
            'module-3-ai-brain/vslam-navigation/exercise-3',
            'module-3-ai-brain/vslam-navigation/examples',
          ],
        },
        {
          type: 'category',
          label: 'Path Planning with Nav2',
          collapsed: true,
          items: [
            'module-3-ai-brain/nav2-path-planning/index',
            'module-3-ai-brain/nav2-path-planning/global-planner',
            'module-3-ai-brain/nav2-path-planning/local-planner',
            'module-3-ai-brain/nav2-path-planning/humanoid-specific',
            'module-3-ai-brain/nav2-path-planning/exercise-4',
            'module-3-ai-brain/nav2-path-planning/examples',
          ],
        },
        {
          type: 'category',
          label: 'Best Practices & Integration',
          collapsed: true,
          items: [
            'module-3-ai-brain/integration-best-practices/index',
            'module-3-ai-brain/integration-best-practices/integration-patterns',
            'module-3-ai-brain/integration-best-practices/performance-optimization',
            'module-3-ai-brain/integration-best-practices/safety-guidelines',
            'module-3-ai-brain/integration-best-practices/dev-workflow',
            'module-3-ai-brain/integration-best-practices/exercise-5',
            'module-3-ai-brain/integration-best-practices/integrated-example',
          ],
        },
      ],
    },
    // Add other modules here as they are created
    {type: 'autogenerated', dirName: '.'},
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
