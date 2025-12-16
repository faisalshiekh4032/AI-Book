import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Sim-to-Real Pipeline',
    emoji: '🎮',
    description: (
      <>
        Master simulation environments like Gazebo and NVIDIA Isaac Sim to train
        robots safely before deploying to physical hardware.
      </>
    ),
  },
  {
    title: 'Industry-Standard Tools',
    emoji: '🛠️',
    description: (
      <>
        Learn ROS 2, the middleware powering commercial robots worldwide, from
        autonomous cars to warehouse automation.
      </>
    ),
  },
  {
    title: 'AI-First Approach',
    emoji: '🧠',
    description: (
      <>
        Integrate LLMs and Vision-Language-Action models to create robots that
        understand natural language commands and perceive their environment.
      </>
    ),
  },
];

function Feature({emoji, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <div className={styles.featureEmoji}>{emoji}</div>
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
