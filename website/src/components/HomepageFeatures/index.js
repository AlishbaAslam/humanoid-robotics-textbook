import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Introduction: Physical AI & Humanoid Robotics',
    path: '/docs/physical-ai-intro/introduction',
    description: (
      <>
        <p>Discover the fundamentals of Physical AI and embodied intelligence. Learn how intelligence emerges from the interaction between an agent and its physical environment, and explore why humanoid robotics offers unique advantages in human-centered environments.</p>
        <ul>
          <li>Understanding Physical AI and embodied intelligence concepts</li>
          <li>Exploring the convergence of AI with physical systems</li>
          <li>Learning about autonomous humanoid robot development</li>
          <li>Understanding real-world robot deployment principles</li>
        </ul>
      </>
    ),
  },
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    path: '/docs/module-1-ros2/intro',
    description: (
      <>
        <p>Learn about the middleware that enables communication between different components of robotic systems. Master ROS 2 fundamentals, Python agent integration, and URDF modeling for humanoid robots.</p>
        <ul>
          <li>Understanding Nodes, Topics, and Services in ROS 2</li>
          <li>Connecting AI agents to ROS 2 using rclpy</li>
          <li>Creating humanoid robot descriptions with URDF</li>
          <li>Building complete robotic communication systems</li>
        </ul>
      </>
    ),
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    path: '/docs/module-2-digital-twin/intro',
    description: (
      <>
        <p>Build digital twins with physics simulation in Gazebo and high-fidelity rendering in Unity, including sensor simulations. Learn to create virtual representations that mirror real-world robot behavior.</p>
        <ul>
          <li>Physics simulation and environment building in Gazebo</li>
          <li>High-fidelity rendering and human-robot interaction in Unity</li>
          <li>Sensor simulation (LiDAR, depth cameras, IMUs) and integration</li>
          <li>Creating comprehensive digital twin systems</li>
        </ul>
      </>
    ),
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    path: '/docs/module-3-ai-robot-brain/intro',
    description: (
      <>
        <p>Explore the NVIDIA Isaac ecosystem for developing intelligent robotic systems. Learn to leverage simulation, perception, and navigation technologies to build advanced AI-powered robots.</p>
        <ul>
          <li>Photorealistic simulation with NVIDIA Isaac Sim</li>
          <li>Hardware-accelerated perception using Isaac ROS</li>
          <li>Navigation2 (Nav2) for humanoid robot path planning</li>
          <li>Synthetic data generation for AI model training</li>
        </ul>
      </>
    ),
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    path: '/docs/module-4-vla-educational-module/intro',
    description: (
      <>
        <p>Discover cutting-edge VLA systems where machines perceive their environment (Vision), understand human instructions (Language), and perform complex physical tasks (Action) for intuitive human-robot interaction.</p>
        <ul>
          <li>Voice-to-Action conversion using OpenAI Whisper</li>
          <li>Cognitive planning with Large Language Models (LLMs)</li>
          <li>Integration of vision, language, and action systems</li>
          <li>Complete VLA system implementation for humanoid robots</li>
        </ul>
      </>
    ),
  },
];

function Feature({title, path, description}) {
  // Extract paragraph and list from the description JSX
  const children = description.props.children;
  const paragraphContent = Array.isArray(children) ? children[0]?.props?.children : children?.props?.children;
  const listContent = Array.isArray(children) ? children[1]?.props?.children : null;

  return (
    <div className={styles.featureCard}>
        <Heading as="h3">{title}</Heading>
        <div className={styles.descriptionContainer}>
          <p className={styles.descriptionText}>{paragraphContent}</p>
          {listContent && <ul className={styles.descriptionList}>{listContent}</ul>}
        </div>
        <div className={styles.buttonContainer}>
          <Link
            className="button button--secondary button--md"
            to={path}>
            Learn More
          </Link>
        </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={clsx(styles.features, styles.moduleFeatures)}>
      <div className="container">
        <div className={styles.gridContainer}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
