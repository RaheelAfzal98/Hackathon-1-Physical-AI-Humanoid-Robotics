---
title: Home
---

import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './index.module.css';

<div className={styles.hero}>
  <div className={styles.heroInner}>
    <h1 className={styles.heroProjectTagline}>
      <span className={styles.heroTitleTextHtml}>
        Physical AI & <b>Humanoid Robotics</b>
      </span>
    </h1>
    <p className={styles.heroSubtitle}>
      An AI-native university textbook on Physical AI, embodied intelligence, and humanoid robotics
    </p>
    <div className={styles.indexCtas}>
      <Link className={styles.indexCtasHeroButton} to="/docs/intro">
        Read the Textbook
      </Link>
      <Link className={styles.indexCtasGitHubButton} href="https://github.com/your-repo">
        GitHub
      </Link>
    </div>
  </div>
</div>

<div className={styles.featuresSection}>
  <div className={styles.featuresContainer}>
    <h2>Explore the Four Core Modules</h2>
    <div className={styles.featuresGrid}>
      <div className={styles.featureCard}>
        <div className={`${styles.featureIcon} ${styles.rosIcon}`}></div>
        <h3>ROS 2 Nervous System</h3>
        <p>Learn about the Robot Operating System 2 and how it functions as the nervous system for humanoid robots.</p>
        <Link to="/docs/module-1-ros2/chapter-1-introduction">Explore Module</Link>
      </div>
      <div className={styles.featureCard}>
        <div className={`${styles.featureIcon} ${styles.digitalTwinIcon}`}></div>
        <h3>The Digital Twin</h3>
        <p>Discover how Gazebo and Unity create digital twins for robotics simulation and development.</p>
        <Link to="/docs/module-2-digital-twin/chapter-1-introduction">Explore Module</Link>
      </div>
      <div className={styles.featureCard}>
        <div className={`${styles.featureIcon} ${styles.aiBrainIcon}`}></div>
        <h3>The AI-Robot Brain</h3>
        <p>Understand NVIDIA Isaac platform and how AI powers the decision-making capabilities of robots.</p>
        <Link to="/docs/module-3-ai-robot-brain/chapter-1-introduction">Explore Module</Link>
      </div>
      <div className={styles.featureCard}>
        <div className={`${styles.featureIcon} ${styles.vlaIcon}`}></div>
        <h3>Vision-Language-Action</h3>
        <p>Explore Vision-Language-Action models that enable robots to perceive, understand, and act.</p>
        <Link to="/docs/module-4-vla/chapter-1-introduction">Explore Module</Link>
      </div>
    </div>
  </div>
</div>

<div className={styles.chatSection}>
  <div className={styles.chatContainer}>
    <h2>AI-Powered Learning Assistant</h2>
    <p>Ask questions about the textbook content and get intelligent answers powered by our RAG system</p>
    <div className={styles.chatWidget}>
      <div className={styles.chatHeader}>
        <h3>🤖 Textbook Assistant</h3>
        <span className={styles.chatStatus}>● Online</span>
      </div>
      <div className={styles.chatPreview}>
        <p>Try asking: "What is ROS 2?" or "Explain digital twins in robotics"</p>
      </div>
      <Link className={styles.chatButton} to="/docs/intro">
        Start Chatting
      </Link>
    </div>
  </div>
</div>