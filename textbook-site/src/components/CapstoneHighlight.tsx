import React from 'react';

export default function CapstoneHighlight() {
  return (
    <section className="bg-linear-to-r from-gray-900 via-gray-800 to-gray-900 py-24">
      <div className="container mx-auto px-6 text-center">
        {/* Heading */}
        <h2 className="text-4xl md:text-5xl font-extrabold bg-linear-to-r from-indigo-400 via-purple-400 to-pink-400 bg-clip-text text-transparent mb-10">
          Capstone Project: Autonomous Humanoid
        </h2>

        {/* Image / Diagram */}
        <div className="flex justify-center mb-10">
          <div className="w-full max-w-2xl h-64 rounded-xl flex items-center justify-center shadow-xl hover:shadow-2xl transition-all duration-500 border-4 border-gray-600 hover:border-indigo-500 overflow-hidden">
            <img
              src="https://www.slideteam.net/media/catalog/product/cache/1280x720/a/r/architecture_for_connected_autonomous_vehicles_iot_in_telecommunications_data_iot_ss_slide01.jpg"
              alt="Autonomous Humanoid Diagram"
              className="w-full h-full object-cover rounded-xl"
            />
          </div>
        </div>

        {/* Description */}
        <p className="text-lg md:text-xl text-gray-300 max-w-3xl mx-auto leading-relaxed">
          Integrate all learned concepts into an{' '}
          <span className="text-indigo-400 font-semibold">end-to-end autonomous humanoid system</span>, capable of understanding voice commands, perceiving its environment, and navigating to perform tasks with intelligence and precision.
        </p>

        {/* CTA Button */}
        <div className="mt-8">
          <a
            href="/docs/capstone/end-to-end-project"
            className="inline-block bg-indigo-600 hover:bg-indigo-500 text-white font-semibold py-3 px-6 rounded-lg shadow-lg transition-all duration-300"
          >
            Learn More
          </a>
        </div>
      </div>
    </section>
  );
}
