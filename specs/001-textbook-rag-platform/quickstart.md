# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Platform

## Prerequisites

- Python 3.11+ with pip
- Node.js 18+ with npm
- Docker (for local development)
- ROS2 Humble Hawksbill (for robotics examples)
- An OpenAI API key

## Setup

### 1. Clone and Initialize Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup

```bash
cd backend
pip install -r requirements.txt
cp .env.example .env
# Edit .env with your API keys and database URLs
```

### 3. Frontend Setup

```bash
cd frontend
npm install
cp .env.example .env
# Edit .env with your API endpoints
```

### 4. Environment Variables

Create `.env` files in both backend and frontend directories:

**Backend (.env):**
```
OPENAI_API_KEY=your_openai_key
NEON_DB_URL=your_neon_db_url
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
SECRET_KEY=your_secret_key
```

**Frontend (.env):**
```
REACT_APP_API_BASE_URL=http://localhost:8000
REACT_APP_AUTH_URL=http://localhost:8000
```

## Running the Application

### 1. Start Backend Services

```bash
# In backend directory
python -m src.api.main
```

### 2. Start Frontend

```bash
# In frontend directory
npm start
```

### 3. Initialize Content

```bash
# In backend directory
python -c "from src.services.rag_service import initialize_content; initialize_content()"
```

## Key Endpoints

### Authentication
- `POST /api/auth/login` - User login
- `POST /api/auth/register` - User registration
- `GET /api/auth/me` - Get current user

### RAG Service
- `POST /api/rag/query` - Query the textbook knowledge base
- `GET /api/rag/status` - Check RAG service status

### Personalization
- `GET /api/personalize/profile` - Get user profile
- `PUT /api/personalize/profile` - Update user profile
- `POST /api/personalize/content` - Get personalized content

### Translation
- `POST /api/translate-ur` - Translate content to Urdu

## Development

### Running Tests

**Backend:**
```bash
pytest tests/
```

**Frontend:**
```bash
npm test
```

### Adding New Content

1. Add new chapters to the `frontend/docs/` directory following the module structure
2. Run the content ingestion script: `python -c "from src.services.rag_service import ingest_new_content; ingest_new_content()"`
3. Update the vector database with the new content

### Adding New Skills

Skills are implemented as modular functions that can be called by subagents. To add a new skill:

1. Create a new function in the `src/utils/skills/` directory
2. Register the skill in the skills registry
3. Update the relevant subagent to use the new skill