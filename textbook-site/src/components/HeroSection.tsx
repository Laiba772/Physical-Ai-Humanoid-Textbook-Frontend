import React from 'react';
import Link from '@docusaurus/Link';

export default function HeroSection() {
  return (
    <section className="relative bg-gray-900 text-white overflow-hidden">
      {/* Optional animated background circles */}
      <div className="absolute top-0 left-1/2 transform -translate-x-1/2 -translate-y-1/2 w-[800px] h-[800px] bg-linear-to-r from-indigo-600 via-purple-600 to-pink-500 opacity-20 rounded-full blur-3xl animate-pulse"></div>
      <div className="container mx-auto px-6 py-32 relative z-10 text-center">
        {/* Heading */}
        <h1 className="text-4xl md:text-6xl font-extrabold leading-tight bg-linear-to-r from-indigo-400 via-purple-400 to-pink-400 bg-clip-text text-transparent">
          Physical AI & Humanoid Robotics
        </h1>

        {/* Subheading */}
        <p className="mt-6 text-lg md:text-xl text-gray-300 max-w-3xl mx-auto leading-relaxed">
          Explore, Simulate, and Build the Future of Intelligent Machines. From ROS 2 and humanoid control to photorealistic simulation, AI-driven perception, and Vision-Language-Action pipelines—this textbook guides you step by step to master the full stack of next-generation robotics.
        </p>

        {/* CTA Button */}
        <div className="mt-10">
          <Link
            to="docs/introduction/intro"
            className="inline-block bg-indigo-600 hover:bg-indigo-500 text-white font-bold py-4 px-8 rounded-xl shadow-lg shadow-indigo-500/50 hover:shadow-indigo-400/60 transition-all duration-300 transform hover:-translate-y-1 hover:scale-105"
          >
            Start Learning {'->'}
          </Link>
        </div>
      </div>
    </section>
  );
}
