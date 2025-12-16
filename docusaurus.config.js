// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const {themes} = require('prism-react-renderer');
const lightTheme = themes.github;
const darkTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-native textbook for embodied intelligence',
  favicon: 'img/favicon.ico',

  url: 'https://yourusername.github.io',
  baseUrl: '/',

  organizationName: 'yourusername',
  projectName: 'physical-ai-robotics-book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  customFields: {
    chatbotApiUrl: process.env.REACT_APP_CHATBOT_API || '',
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs',
          editUrl: 'https://github.com/yourusername/physical-ai-robotics-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.png',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/yourusername/physical-ai-robotics-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Course Overview',
                to: '/docs/overview/about',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'Discord',
                href: 'https://discord.gg/panaversity',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/yourusername/physical-ai-robotics-book',
              },
              {
                label: 'Panaversity',
                href: 'https://panaversity.org',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Meta-Cognition Lab. Built with Docusaurus and Spec-First AI.`,
      },
      prism: {
        theme: lightTheme,
        darkTheme: darkTheme,
        additionalLanguages: ['python', 'bash', 'yaml', 'cpp', 'xml', 'json'],
      },
    }),
};

module.exports = config;
