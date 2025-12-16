"""
FastAPI-based RAG chatbot for Physical AI & Humanoid Robotics course.
Integrates with OpenAI, Qdrant, and Neon Postgres.
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import os
from dotenv import load_dotenv
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import psycopg
import hashlib
import uuid

load_dotenv()

app = FastAPI(
    title="Physical AI Robotics RAG Chatbot",
    description="Retrieval-Augmented Generation chatbot for the course book",
    version="1.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Qdrant client (optional if deployed)
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
if QDRANT_URL and QDRANT_API_KEY:
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    COLLECTION_NAME = "robotics_course"
else:
    qdrant_client = None
    COLLECTION_NAME = None

# Request/Response models
class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    conversation_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[str] = []
    conversation_id: str

# Database connection
def get_db_connection():
    db_url = os.getenv("NEON_DATABASE_URL")
    if db_url:
        return psycopg.connect(db_url)
    return None

# Initialize database tables
@app.on_event("startup")
async def startup_event():
    """Initialize database schema on startup"""
    conn = get_db_connection()
    if conn:
        try:
            with conn.cursor() as cur:
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS conversations (
                        id SERIAL PRIMARY KEY,
                        conversation_id VARCHAR(64) UNIQUE,
                        created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                    )
                """)
                cur.execute("""
                    CREATE TABLE IF NOT EXISTS messages (
                        id SERIAL PRIMARY KEY,
                        conversation_id VARCHAR(64),
                        role VARCHAR(20),
                        content TEXT,
                        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        FOREIGN KEY (conversation_id) REFERENCES conversations(conversation_id)
                    )
                """)
                conn.commit()
        finally:
            conn.close()

# Embedding function
def get_embedding(text: str) -> List[float]:
    """Generate embeddings using OpenAI"""
    response = openai_client.embeddings.create(
        model="text-embedding-3-small",
        input=text
    )
    return response.data[0].embedding

# Retrieve relevant context
def retrieve_context(query: str, top_k: int = 3) -> tuple[str, List[str]]:
    """Retrieve relevant context from Qdrant vector database"""
    if not qdrant_client or not COLLECTION_NAME:
        return "", []
    
    try:
        query_vector = get_embedding(query)
        
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=top_k
        )
        
        contexts = []
        sources = []
        for hit in search_result:
            contexts.append(hit.payload.get("text", ""))
            sources.append(hit.payload.get("source", "Unknown"))
        
        return "\n\n".join(contexts), sources
    except Exception as e:
        print(f"Error retrieving context: {e}")
        return "", []

# Save conversation to database
def save_message(conversation_id: str, role: str, content: str):
    """Save message to Neon Postgres database"""
    conn = get_db_connection()
    if not conn:
        return
    
    try:
        with conn.cursor() as cur:
            # Ensure conversation exists
            cur.execute("""
                INSERT INTO conversations (conversation_id)
                VALUES (%s)
                ON CONFLICT (conversation_id) DO NOTHING
            """, (conversation_id,))
            
            # Insert message
            cur.execute("""
                INSERT INTO messages (conversation_id, role, content)
                VALUES (%s, %s, %s)
            """, (conversation_id, role, content))
            
            conn.commit()
    finally:
        conn.close()

@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "online",
        "service": "Physical AI Robotics Chatbot API",
        "qdrant_enabled": qdrant_client is not None,
        "database_enabled": os.getenv("NEON_DATABASE_URL") is not None
    }

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Process chat request with RAG (Retrieval-Augmented Generation).
    
    - Retrieves relevant context from vector database
    - Uses OpenAI GPT for response generation
    - Saves conversation history
    """
    try:
        # Generate or use existing conversation ID
        conversation_id = request.conversation_id or str(uuid.uuid4())
        
        # Save user message
        save_message(conversation_id, "user", request.message)
        
        # Determine context
        if request.selected_text:
            # User selected specific text
            context = request.selected_text
            sources = ["Selected text"]
        else:
            # Retrieve from vector database
            context, sources = retrieve_context(request.message)
        
        # Build system prompt
        system_prompt = """You are an AI assistant for a Physical AI and Humanoid Robotics course.
        You help students understand concepts related to ROS 2, Gazebo simulation, NVIDIA Isaac,
        Vision-Language-Action systems, and humanoid robot development.
        
        Answer questions clearly and concisely. Use the provided context when available.
        If you don't know something, say so rather than making up information.
        """
        
        # Build user prompt
        if context:
            user_prompt = f"""Context from the course materials:
{context}

Student question: {request.message}

Please answer based on the context provided. If the context doesn't contain relevant information, provide a general answer based on your knowledge of robotics and AI."""
        else:
            user_prompt = request.message
        
        # Generate response using OpenAI
        completion = openai_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=500
        )
        
        assistant_message = completion.choices[0].message.content
        
        # Save assistant response
        save_message(conversation_id, "assistant", assistant_message)
        
        return ChatResponse(
            response=assistant_message,
            sources=sources if sources else [],
            conversation_id=conversation_id
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing request: {str(e)}")

@app.get("/health")
async def health_check():
    """Detailed health check"""
    health = {
        "api": "healthy",
        "openai": "unknown",
        "qdrant": "disabled" if not qdrant_client else "unknown",
        "database": "disabled" if not os.getenv("NEON_DATABASE_URL") else "unknown"
    }
    
    # Check OpenAI
    try:
        openai_client.models.list()
        health["openai"] = "healthy"
    except:
        health["openai"] = "unhealthy"
    
    # Check Qdrant
    if qdrant_client:
        try:
            qdrant_client.get_collections()
            health["qdrant"] = "healthy"
        except:
            health["qdrant"] = "unhealthy"
    
    # Check database
    conn = get_db_connection()
    if conn:
        try:
            with conn.cursor() as cur:
                cur.execute("SELECT 1")
            health["database"] = "healthy"
        except:
            health["database"] = "unhealthy"
        finally:
            conn.close()
    
    return health

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
