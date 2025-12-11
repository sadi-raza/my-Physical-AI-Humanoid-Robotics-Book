"""Project-wide configuration settings and environment variables."""

import os
from typing import Optional
from pydantic import BaseModel
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings(BaseModel):
    """Application settings loaded from environment variables."""

    # Server settings
    app_name: str = "Physical AI & Humanoid Robotics Textbook Platform"
    app_version: str = "1.0.0"
    host: str = os.getenv("HOST", "0.0.0.0")
    port: int = int(os.getenv("PORT", "8000"))
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"

    # Database settings
    neon_db_url: str = os.getenv("NEON_DB_URL", "postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require")
    qdrant_url: str = os.getenv("QDRANT_URL", "https://your-cluster-url.qdrant.tech:6333")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_host: str = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port: int = int(os.getenv("QDRANT_PORT", "6333"))

    # OpenAI settings
    openai_api_key: Optional[str] = os.getenv("OPENAI_API_KEY")
    openai_model: str = os.getenv("OPENAI_MODEL", "gpt-4-turbo")

    # Better-Auth settings
    auth_secret: str = os.getenv("AUTH_SECRET", "your-super-secret-jwt-key-change-this")
    auth_algorithm: str = os.getenv("AUTH_ALGORITHM", "HS256")
    auth_access_token_expire_minutes: int = int(os.getenv("AUTH_ACCESS_TOKEN_EXPIRE_MINUTES", "30"))
    auth_refresh_token_expire_days: int = int(os.getenv("AUTH_REFRESH_TOKEN_EXPIRE_DAYS", "7"))

    # RAG settings
    rag_max_response_time_ms: int = int(os.getenv("RAG_MAX_RESPONSE_TIME_MS", "800"))
    rag_confidence_threshold: float = float(os.getenv("RAG_CONFIDENCE_THRESHOLD", "0.7"))
    rag_top_k_results: int = int(os.getenv("RAG_TOP_K_RESULTS", "5"))

    # Content settings
    content_grade_level_min: int = int(os.getenv("CONTENT_GRADE_LEVEL_MIN", "10"))
    content_grade_level_max: int = int(os.getenv("CONTENT_GRADE_LEVEL_MAX", "12"))

    # Translation settings
    urdu_translation_enabled: bool = os.getenv("URDU_TRANSLATION_ENABLED", "True").lower() == "true"

    # Security settings
    allowed_origins: list = os.getenv("ALLOWED_ORIGINS", "http://localhost,http://localhost:3000,http://127.0.0.1,http://127.0.0.1:3000").split(",")

    # Logging settings
    log_level: str = os.getenv("LOG_LEVEL", "INFO")
    log_format: str = os.getenv("LOG_FORMAT", "%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    # Cache settings
    cache_ttl_seconds: int = int(os.getenv("CACHE_TTL_SECONDS", "3600"))  # 1 hour default

    # Vector settings
    vector_embedding_model: str = os.getenv("VECTOR_EMBEDDING_MODEL", "text-embedding-3-small")
    vector_collection_name: str = os.getenv("VECTOR_COLLECTION_NAME", "textbook_content")

    # Personalization settings
    personalization_enabled: bool = os.getenv("PERSONALIZATION_ENABLED", "True").lower() == "true"

    class Config:
        env_file = ".env"
        case_sensitive = True

# Create a global settings instance
settings = Settings()