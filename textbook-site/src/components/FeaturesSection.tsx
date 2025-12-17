import React from 'react';

const features = [
  {
    title: 'RAG AI Assistant',
    description: 'An AI assistant powered by Retrieval-Augmented Generation to help you with your questions.',
    icon: '💬',
    color: 'bg-indigo-500',
  },
  {
    title: 'Personalization',
    description: 'Tailor your learning path with personalized content and quizzes.',
    icon: '✨',
    color: 'bg-pink-500',
  },
  {
    title: 'Urdu Translation',
    description: 'Read the textbook in Urdu with a single click.',
    icon: '🌍',
    color: 'bg-green-500',
  },
  {
    title: 'Simulation-based Learning',
    description: 'Learn by doing with our simulation-focused examples. No hardware required.',
    icon: '🔬',
    color: 'bg-yellow-500',
  },
];

export default function FeaturesSection() {
  return (
    <section className="bg-gray-900 py-24">
      <div className="container mx-auto px-6">
        <h2 className="text-4xl md:text-5xl font-extrabold text-center text-white mb-16">
          Features
        </h2>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-10">
          {features.map((feature) => (
            <div
              key={feature.title}
              className="bg-linear-to-r from-gray-800 via-gray-900 to-gray-800 p-8 rounded-xl transform hover:-translate-y-2 hover:scale-105 transition-all duration-500 shadow-2xl"
            >
              <div className={`w-16 h-16 flex items-center justify-center text-3xl mb-6 rounded-full text-white ${feature.color} shadow-lg`}>
                {feature.icon}
              </div>
              <h3 className="text-2xl font-bold text-white mb-3">{feature.title}</h3>
              <p className="text-gray-300 leading-relaxed">{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
