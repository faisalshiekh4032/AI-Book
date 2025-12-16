# Physical AI & Humanoid Robotics Course Book

An AI-native textbook for learning Physical AI and Humanoid Robotics, built with Docusaurus and integrated with RAG chatbot capabilities.

## 🚀 Features

- **Comprehensive Curriculum**: 13-week course covering ROS 2, Gazebo, NVIDIA Isaac, and VLA systems
- **Interactive Learning**: Embedded AI chatbot for instant answers
- **Modern Stack**: Built with Docusaurus, FastAPI, OpenAI, Qdrant, and Neon Postgres
- **Responsive Design**: Mobile-friendly interface
- **Production Ready**: Optimized for deployment on Vercel or GitHub Pages

## 📚 Course Modules

### Module 1: ROS 2 Fundamentals (Weeks 3-5)
- ROS 2 architecture and DDS middleware
- Nodes, topics, services, and actions
- Python integration with rclpy
- URDF robot modeling

### Module 2: Digital Twin & Simulation (Weeks 6-7)
- Gazebo simulation environment
- Unity integration for visualization
- Sensor simulation (LIDAR, cameras, IMUs)
- Physics-based testing

### Module 3: NVIDIA Isaac Platform (Weeks 8-10)
- Isaac Sim for photorealistic simulation
- Isaac ROS for GPU-accelerated perception
- Visual SLAM and navigation
- Nav2 for autonomous movement

### Module 4: Vision-Language-Action (Weeks 11-13)
- Voice control with OpenAI Whisper
- LLM-based task planning
- Multimodal AI integration
- Capstone: Autonomous humanoid assistant

## 🛠️ Tech Stack

**Frontend**:
- Docusaurus 3.5.2
- React 18
- Custom CSS modules

**Backend** (Optional - for chatbot):
- FastAPI
- OpenAI GPT-4
- Qdrant Vector Database
- Neon Serverless Postgres

## 🚀 Quick Start

### Prerequisites

- Node.js 18+ and npm
- Git

### Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/physical-ai-robotics-book.git
cd physical-ai-robotics-book

# Install dependencies
npm install

# Start development server
npm start
```

The site will open at `http://localhost:3000`

### Build for Production

```bash
npm run build
```

Output will be in the `build/` directory.

## 📦 Deployment

### Deploy to Vercel (Recommended)

```bash
# Install Vercel CLI
npm install -g vercel

# Deploy
vercel
```

### Deploy to GitHub Pages

1. Update `docusaurus.config.js`:
   ```js
   url: 'https://yourusername.github.io',
   baseUrl: '/physical-ai-robotics-book/',
   organizationName: 'yourusername',
   projectName: 'physical-ai-robotics-book',
   ```

2. Deploy:
   ```bash
   GIT_USER=yourusername npm run deploy
   ```

## 🤖 Chatbot Setup (Optional)

The integrated RAG chatbot requires backend setup:

### 1. Get API Keys

- [OpenAI API Key](https://platform.openai.com/api-keys)
- [Qdrant Cloud](https://cloud.qdrant.io/) (free tier)
- [Neon Postgres](https://neon.tech/) (free tier)

### 2. Configure Backend

```bash
cd chatbot
cp .env.example .env
# Edit .env with your API keys

# Install Python dependencies
pip install -r requirements.txt

# Run backend
cd api
python main.py
```

### 3. Update Frontend Config

Edit `docusaurus.config.js`:
```js
customFields: {
  chatbotApiUrl: 'https://your-backend-url.com',
},
```

### 4. Generate Embeddings

```bash
cd chatbot/embeddings
python generate_embeddings.py
```

See [chatbot/README.md](chatbot/README.md) for detailed instructions.

## 📖 Documentation Structure

```
docs/
├── intro.md                 # Course introduction
├── overview/                # Course overview and schedule
├── module1/                 # ROS 2 Fundamentals
├── module2/                 # Digital Twin & Simulation
├── module3/                 # NVIDIA Isaac Platform
├── module4/                 # Vision-Language-Action
├── hardware/                # Hardware requirements
└── assessments/             # Projects and evaluation
```

## 🎨 Customization

### Branding

Edit `docusaurus.config.js`:
```js
title: 'Your Course Name',
tagline: 'Your tagline',
favicon: 'img/your-favicon.ico',
```

### Styling

Modify `src/css/custom.css` for colors and styles.

### Content

All course content is in Markdown files under `docs/`.

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is created for educational purposes as part of the Panaversity Hackathon I.

## 🙏 Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Course designed by Meta-Cognition Lab
- Powered by Panaversity initiative

## 📧 Contact

For questions or feedback:
- Discord: [Join Panaversity Discord](https://discord.gg/panaversity)
- GitHub Issues: [Report issues](https://github.com/yourusername/physical-ai-robotics-book/issues)

---

**Made with ❤️ for the robotics community**
