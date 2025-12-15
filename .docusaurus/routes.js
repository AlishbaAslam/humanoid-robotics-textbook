import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', 'b2f'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/authors',
    component: ComponentCreator('/blog/authors', '0b7'),
    exact: true
  },
  {
    path: '/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/blog/authors/all-sebastien-lorber-articles', '4a1'),
    exact: true
  },
  {
    path: '/blog/authors/yangshun',
    component: ComponentCreator('/blog/authors/yangshun', 'a68'),
    exact: true
  },
  {
    path: '/blog/first-blog-post',
    component: ComponentCreator('/blog/first-blog-post', '89a'),
    exact: true
  },
  {
    path: '/blog/long-blog-post',
    component: ComponentCreator('/blog/long-blog-post', '9ad'),
    exact: true
  },
  {
    path: '/blog/mdx-blog-post',
    component: ComponentCreator('/blog/mdx-blog-post', 'e9f'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '704'),
    exact: true
  },
  {
    path: '/blog/tags/facebook',
    component: ComponentCreator('/blog/tags/facebook', '858'),
    exact: true
  },
  {
    path: '/blog/tags/hello',
    component: ComponentCreator('/blog/tags/hello', '299'),
    exact: true
  },
  {
    path: '/blog/tags/hola',
    component: ComponentCreator('/blog/tags/hola', '00d'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', 'd2b'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '068'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'ec3'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '3b3'),
            routes: [
              {
                path: '/docs/module-1-ros2/chapter-1-ros2/',
                component: ComponentCreator('/docs/module-1-ros2/chapter-1-ros2/', 'e19'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-1-ros2/exercise-1-basic-nodes',
                component: ComponentCreator('/docs/module-1-ros2/chapter-1-ros2/exercise-1-basic-nodes', '82f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-1-ros2/introduction',
                component: ComponentCreator('/docs/module-1-ros2/chapter-1-ros2/introduction', 'b11'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-1-ros2/setup',
                component: ComponentCreator('/docs/module-1-ros2/chapter-1-ros2/setup', '70a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-2-python-agents/',
                component: ComponentCreator('/docs/module-1-ros2/chapter-2-python-agents/', '33e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-2-python-agents/exercise-2-python-integration',
                component: ComponentCreator('/docs/module-1-ros2/chapter-2-python-agents/exercise-2-python-integration', '4f9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-3-urdf-modeling/',
                component: ComponentCreator('/docs/module-1-ros2/chapter-3-urdf-modeling/', 'bd6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-3-urdf-modeling/complete_integration',
                component: ComponentCreator('/docs/module-1-ros2/chapter-3-urdf-modeling/complete_integration', '23c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-3-urdf-modeling/exercise-3-urdf-modeling',
                component: ComponentCreator('/docs/module-1-ros2/chapter-3-urdf-modeling/exercise-3-urdf-modeling', '365'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapter-3-urdf-modeling/final_summary',
                component: ComponentCreator('/docs/module-1-ros2/chapter-3-urdf-modeling/final_summary', '450'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/accessibility-guide',
                component: ComponentCreator('/docs/module-2-digital-twin/accessibility-guide', 'bc4'),
                exact: true
              },
              {
                path: '/docs/module-2-digital-twin/assessment-questions',
                component: ComponentCreator('/docs/module-2-digital-twin/assessment-questions', 'fec'),
                exact: true
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1-gazebo/',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1-gazebo/', '75c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1-gazebo/citations',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1-gazebo/citations', '25d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1-gazebo/diagrams-and-screenshots-guide',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1-gazebo/diagrams-and-screenshots-guide', 'dbc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1-gazebo/exercise-1-basic-robot',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1-gazebo/exercise-1-basic-robot', '935'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1-gazebo/gazebo-plugins-examples',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1-gazebo/gazebo-plugins-examples', 'd81'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1-gazebo/gazebo-setup-guide',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1-gazebo/gazebo-setup-guide', '37b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-1-gazebo/troubleshooting-guide',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-1-gazebo/troubleshooting-guide', 'a90'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/', '568'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/diagrams-screenshots-guide',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/diagrams-screenshots-guide', '821'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/exercise-2-unity-visualization',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/exercise-2-unity-visualization', 'cef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/material-lighting-setup',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/material-lighting-setup', 'c37'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/optimization-techniques',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/optimization-techniques', 'fc5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/robot-model-import-setup',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/robot-model-import-setup', 'c2d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/unity-documentation-citations',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/unity-documentation-citations', '115'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/unity-installation-setup',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/unity-installation-setup', '8a1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/unity-ros-integration-examples',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/unity-ros-integration-examples', '6c9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-2-unity/user-interface-controls',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-2-unity/user-interface-controls', '32e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/', 'ea4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/depth-camera-simulation',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/depth-camera-simulation', 'aad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/exercise-3-sensor-simulation',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/exercise-3-sensor-simulation', '35c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/imu-simulation',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/imu-simulation', '8af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/lidar-simulation-tutorial',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/lidar-simulation-tutorial', '916'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/sensor-data-processing-ros',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/sensor-data-processing-ros', '72d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/sensor-data-validation-comparison',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/sensor-data-validation-comparison', '8a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/sensor-fusion-digital-twin',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/sensor-fusion-digital-twin', 'ebf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/sensor-simulation-diagrams',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/sensor-simulation-diagrams', '027'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/chapter-3-sensors/sensor-simulation-research-citations',
                component: ComponentCreator('/docs/module-2-digital-twin/chapter-3-sensors/sensor-simulation-research-citations', 'a34'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/complete-digital-twin-example',
                component: ComponentCreator('/docs/module-2-digital-twin/complete-digital-twin-example', 'f05'),
                exact: true
              },
              {
                path: '/docs/module-2-digital-twin/cross-chapter-exercises',
                component: ComponentCreator('/docs/module-2-digital-twin/cross-chapter-exercises', '623'),
                exact: true
              },
              {
                path: '/docs/module-2-digital-twin/digital-twin-citations',
                component: ComponentCreator('/docs/module-2-digital-twin/digital-twin-citations', '8b1'),
                exact: true
              },
              {
                path: '/docs/module-2-digital-twin/integration-guide',
                component: ComponentCreator('/docs/module-2-digital-twin/integration-guide', 'ab1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-digital-twin/summary-and-next-steps',
                component: ComponentCreator('/docs/module-2-digital-twin/summary-and-next-steps', '8b2'),
                exact: true
              },
              {
                path: '/docs/module-2-digital-twin/troubleshooting-integration',
                component: ComponentCreator('/docs/module-2-digital-twin/troubleshooting-integration', '99c'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/', '92c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/citations',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/citations', 'ba1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/code-examples',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/code-examples', '910'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/cross-platform-considerations',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/cross-platform-considerations', 'aba'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/diagrams',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/diagrams', '42f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/environment-creation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/environment-creation', '0cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/exercise-1',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/exercise-1', '631'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/exercise-template',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/exercise-template', 'e51'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/integration-examples',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/integration-examples', '2a1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/setup-guide',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/setup-guide', 'c6b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/synthetic-data-generation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/synthetic-data-generation', 'c41'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/template',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/template', 'cb7'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/troubleshooting',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-1-isaac-sim/troubleshooting', '82a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/', 'a9d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/assessment',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/assessment', 'd49'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/code-examples',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/code-examples', 'df8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/component-configuration',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/component-configuration', '7c1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/diagrams',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/diagrams', '6e7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/exercise-2',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/exercise-2', '861'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/glossary',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/glossary', '96b'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/navigation-examples',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/navigation-examples', '2d9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/setup-guide',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/setup-guide', '0f7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/summary',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/summary', '5da'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/troubleshooting',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/troubleshooting', '365'),
                exact: true
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/vslam-implementation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-2-isaac-ros/vslam-implementation', 'cf8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-3-nav2-humanoid/',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-3-nav2-humanoid/', 'b6b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-3-nav2-humanoid/humanoid-config',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-3-nav2-humanoid/humanoid-config', '8a5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/chapter-3-nav2-humanoid/setup-guide',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/chapter-3-nav2-humanoid/setup-guide', '10c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-educational-module/chapter-1-voice-to-action/',
                component: ComponentCreator('/docs/module-4-vla-educational-module/chapter-1-voice-to-action/', '332'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-educational-module/chapter-2-cognitive-planning/',
                component: ComponentCreator('/docs/module-4-vla-educational-module/chapter-2-cognitive-planning/', 'f41'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla-educational-module/chapter-3-capstone-project/',
                component: ComponentCreator('/docs/module-4-vla-educational-module/chapter-3-capstone-project/', 'eef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/physical-ai-intro/introduction',
                component: ComponentCreator('/docs/physical-ai-intro/introduction', 'd62'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
