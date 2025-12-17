import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline:
    'Explore, Simulate, and Build the Future of Intelligent Machines. From ROS 2 and humanoid control to photorealistic simulation, AI-driven perception, and Vision-Language-Action pipelines—this textbook guides you step by step to master the full stack of next-generation robotics.',
  favicon: 'https://tse1.mm.bing.net/th/id/OIP.yXjnRTsZq8qVU9F48yCjSwHaHM?rs=1&pid=ImgDetMain&o=7&rm=3',
  future: {
    v4: true,
  },

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'your-org',
  projectName: 'humanoid-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',


  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
    },
  },

  plugins: ['./src/plugins/tailwind-config.js'],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/your-org/humanoid-textbook/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics Textbook',
      logo: {
        alt: 'My Site Logo',
        src: 'https://tse1.mm.bing.net/th/id/OIP.yXjnRTsZq8qVU9F48yCjSwHaHM?rs=1&pid=ImgDetMain&o=7&rm=3',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/Laiba772/Physical-Ai-Humanoid-Textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
  style: 'dark',
  links: [
    {
      title: '🌐 Socials',
      items: [
        {
          label: '🐙 GitHub',
          href: 'https://github.com/Laiba772',
          className:
            'transition-all hover:bg-clip-text hover:text-transparent hover:bg-gradient-to-r hover:from-purple-400 hover:to-pink-500',
        },
        {
          label: '💼 LinkedIn',
          href: 'https://www.linkedin.com/in/laiba-naz-643b192b5/',
          className:
            'transition-all hover:bg-clip-text hover:text-transparent hover:bg-gradient-to-r hover:from-blue-400 hover:to-cyan-400',
        },
        {
          label: '🐦 Twitter',
          href: 'https://x.com/RajLaiba',
          className:
            'transition-all hover:bg-clip-text hover:text-transparent hover:bg-gradient-to-r hover:from-sky-400 hover:to-blue-500',
        },
        {
          label: '📸 Instagram',
          href: 'https://www.instagram.com/laibanaz012/',
          className:
            'transition-all hover:bg-clip-text hover:text-transparent hover:bg-gradient-to-r hover:from-pink-400 hover:to-yellow-400',
        },
        {
          label: '▶️ YouTube',
          href: 'https://www.youtube.com/@motivate-l9v',
          className:
            'transition-all hover:bg-clip-text hover:text-transparent hover:bg-gradient-to-r hover:from-red-400 hover:to-orange-500',
        },
      ],
    },

    // ⭐ NEW UNIQUE SECTION - ABOUT
    {
      title: '📘 About This Project',
      items: [
        {
          label: 'What is Physical AI?',
          to: 'docs/introduction/intro',
          className: 'hover:text-purple-300 transition-colors',
        },
        {
          label: 'Why Humanoid Robotics?',
          to: 'docs/introduction/why-humanoids',
          className: 'hover:text-pink-300 transition-colors',
        },
        {
          label: 'For Students & Researchers',
          to: 'docs/introduction/for-students',
          className: 'hover:text-blue-300 transition-colors',
        },
      ],
    },

    // ⭐ NEW SECTION - QUICK LINKS
    {
      title: '⚡ Quick Links',
      items: [
        {
          label: '🚀 Start Learning',
          to: 'docs/introduction/intro',
          className: 'hover:underline hover:text-blue-400 transition-colors',
        },
        {
          label: '📂 GitHub Repo',
          href: 'https://github.com/your-org/humanoid-textbook',
          className: 'hover:underline hover:text-green-300 transition-colors',
        },
        {
          label: '📨 Contact / Support',
          href: 'mailto:laiba.robostudy@gmail.com',
          className: 'hover:underline hover:text-yellow-300 transition-colors',
        },
      ],
    },

    // ⭐ NEW SECTION - RESOURCES
    {
      title: '🛠️ Resources',
      items: [
        {
          label: '🤖 Robotics Roadmap',
          href: 'https://roadmap.sh/ai',
          className: 'hover:text-teal-300 transition-colors',
        },
        {
          label: '🎓 ROS 2 Learning Path',
          href: 'https://docs.ros.org/',
          className: 'hover:text-orange-300 transition-colors',
        },
        {
          label: '🧠 AI Learning Path',
          href: 'https://www.deeplearning.ai/',
          className: 'hover:text-purple-300 transition-colors',
        },
      ],
    },
  ],

  copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with ❤️ and 💡 by Laiba Naz — Empowering the Next Generation of Robotics Innovators.`,
},


    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;