# RAG Chatbot Backend

This directory contains the FastAPI-based RAG (Retrieval-Augmented Generation) chatbot backend for the Physical AI & Humanoid Robotics course.

## Features

- **OpenAI Integration**: Uses GPT-4 for intelligent responses
- **Vector Search**: Qdrant Cloud for semantic search over course content
- **Conversation History**: Neon Serverless Postgres for persistent storage
- **Selected Text Support**: Can answer questions about specific highlighted text

## Setup

### Prerequisites

- Python 3.10+
- OpenAI API key
- Qdrant Cloud account (free tier)
- Neon Serverless Postgres database (free tier)

### Installation

```bash
cd chatbot
python -m venv venv
source venv/bin/activate  # On Windows: venv\\Scripts\\activate
pip install -r requirements.txt
```

### Configuration

Create a `.env` file:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```
OPENAI_API_KEY=sk-your-key-here
NEON_DATABASE_URL=postgres://user:pass@host/db?sslmode=require
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
```

### Run Locally

```bash
cd api
python main.py
```

Or with uvicorn:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

API will be available at `http://localhost:8000`

### API Documentation

Interactive API docs at `http://localhost:8000/docs`

## Deployment

### Option 1: Vercel (Recommended)

```bash
# Install Vercel CLI
npm install -g vercel

# Deploy
cd chatbot
vercel
```

### Option 2: Railway

1. Create a new project on Railway
2. Connect your GitHub repository
3. Set environment variables
4. Deploy

### Option 3: Docker

```bash
docker build -t robotics-chatbot .
docker run -p 8000:8000 --env-file .env robotics-chatbot
```

## Embeddings Generation

To populate the Qdrant vector database with course content:

```bash
cd embeddings
python generate_embeddings.py
```

This script:
1. Reads all markdown files from the docs directory
2. Chunks content into manageable pieces
3. Generates embeddings using OpenAI
4. Uploads to Qdrant Cloud

## API Endpoints

### POST /chat

Main chatbot endpoint.

**Request**:
```json
{
  "message": "What is ROS 2?",
  "selected_text": "optional highlighted text",
  "conversation_id": "optional-uuid"
}
```

**Response**:
```json
{
  "response": "ROS 2 is...",
  "sources": ["Module 1: ROS 2 Introduction"],
  "conversation_id": "uuid-here"
}
```

### GET /health

Health check endpoint.

### GET /

Service status.

## Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `OPENAI_API_KEY` | OpenAI API key for GPT-4 and embeddings | Yes |
| `NEON_DATABASE_URL` | Postgres connection string | Optional* |
| `QDRANT_URL` | Qdrant Cloud instance URL | Optional* |
| `QDRANT_API_KEY` | Qdrant API key | Optional* |

*If not provided, chatbot will work without RAG (no context retrieval) and without conversation history.

## Architecture

```
┌─────────────┐
│  Docusaurus │
│   Frontend  │
└──────┬──────┘
       │ HTTP
       ↓
┌─────────────┐      ┌────────────┐
│   FastAPI   │─────→│  OpenAI    │
│   Backend   │      │  GPT-4     │
└──────┬──────┘      └────────────┘
       │
       ├───→ ┌────────────┐
       │     │   Qdrant   │
       │     │   Vector   │
       │     │     DB     │
       │     └────────────┘
       │
       └───→ ┌────────────┐
             │    Neon    │
             │  Postgres  │
             └────────────┘
```

## Troubleshooting

### CORS Issues

If you encounter CORS errors, update the `allow_origins` in `main.py`:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://yourdomain.com"],  # Your deployed domain
    ...
)
```

### Connection Errors

Check that all services are accessible:

```bash
curl http://localhost:8000/health
```

### Rate Limits

OpenAI has rate limits. For production, implement caching or use Azure OpenAI.

## License

Part of the Physical AI & Humanoid Robotics course materials.
