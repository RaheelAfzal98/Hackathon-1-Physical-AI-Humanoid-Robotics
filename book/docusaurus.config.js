module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook on Physical AI and Humanoid Robotics',
  url: 'https://your-vercel-project.vercel.app',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'your-username',
  projectName: 'physical-ai-humanoid-robotics',
  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      hideOnScroll: false,
      style: 'dark',
      items: [
        {
          to: '/docs/intro',
          label: 'Textbook',
          position: 'left',
        },
        {
          href: 'https://github.com/your-username/physical-ai-humanoid-robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook Modules',
          items: [
            {
              label: 'Module 1: ROS 2 Nervous System',
              to: '/docs/module-1-ros2/chapter-1-introduction',
            },
            {
              label: 'Module 2: The Digital Twin',
              to: '/docs/module-2-digital-twin/chapter-1-introduction',
            },
            {
              label: 'Module 3: The AI-Robot Brain',
              to: '/docs/module-3-ai-robot-brain/chapter-1-introduction',
            },
            {
              label: 'Module 4: Vision-Language-Action',
              to: '/docs/module-4-vla/chapter-1-introduction',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'ROS Answers',
              href: 'https://answers.ros.org/',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/your-username/physical-ai-humanoid-robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Created with assistance from Qwen Code AI Assistant. Built with Docusaurus.`,
    },
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-username/physical-ai-humanoid-robotics/edit/main/docs/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};