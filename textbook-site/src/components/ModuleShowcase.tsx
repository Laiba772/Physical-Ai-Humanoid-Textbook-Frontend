import React from 'react';
import Link from '@docusaurus/Link';

const modules = [
  {
    title: 'ROS 2',
    description: 'The Robotic Nervous System. Learn the fundamentals of the Robot Operating System.',
    link: 'docs/ros2/ros2-basics',
    icon: '🤖',
    color: 'bg-indigo-500',
  },
  {
    title: 'Digital Twin',
    description: 'Simulate and test in Gazebo & Unity. Master the art of virtual robot replication.',
    link: 'docs/digital-twin/physics-simulation',
    icon: '🌐',
    color: 'bg-green-500',
  },
  {
    title: 'NVIDIA Isaac',
    description: 'The AI-Robot Brain. Dive into perception, VSLAM, and navigation.',
    link: 'docs/isaac/perception-vslam',
    icon: '🧠',
    color: 'bg-pink-500',
  },
  {
    title: 'Vision-Language-Action (VLA)',
    description: 'Build conversational robots with voice-to-action and LLM cognitive planning.',
    link: 'docs/vla/voice-to-action',
    icon: '🗣️',
    color: 'bg-yellow-500',
  },
];

export default function ModuleShowcase() {
  return (
    <section className="bg-gray-900 py-24">
      <div className="container mx-auto px-6">
        <h2 className="text-4xl md:text-5xl font-extrabold text-center text-white mb-16">
          Core Modules
        </h2>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-10">
          {modules.map((module) => (
            <Link
              key={module.title}
              to={module.link}
              className="block p-8 rounded-2xl bg-linear-to-br from-gray-800 via-gray-900 to-gray-800 transform hover:-translate-y-2 hover:scale-105 transition-all duration-500 shadow-2xl border-2 border-transparent hover:border-indigo-400"
            >
              <div
                className={`w-16 h-16 flex items-center justify-center text-3xl mb-6 rounded-full text-white ${module.color} shadow-lg`}
              >
                {module.icon}
              </div>
              <h3 className="text-2xl font-bold text-white mb-3">{module.title}</h3>
              <p className="text-gray-300 leading-relaxed">{module.description}</p>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}
