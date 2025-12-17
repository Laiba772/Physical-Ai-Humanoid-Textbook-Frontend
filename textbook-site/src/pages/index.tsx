import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import HeroSection from '../components/HeroSection';
import ModuleShowcase from '../components/ModuleShowcase';
import CapstoneHighlight from '../components/CapstoneHighlight';
import FeaturesSection from '../components/FeaturesSection';

export default function Home(): React.ReactElement {
  return (
    <Layout title="Physical AI & Humanoid Robotics Textbook">
      <main>
        {/* Hero Section */}
        <HeroSection />

        {/* Core Modules */}
        <ModuleShowcase />

        {/* Capstone Project */}
        <CapstoneHighlight />

        {/* Features Section */}
        <FeaturesSection />
      </main>
    </Layout>
  );
}
