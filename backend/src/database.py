"""Database connection setup for NeonDB (PostgreSQL) and Qdrant vector database clients."""

from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from qdrant_client import QdrantClient
from typing import Optional
from contextlib import contextmanager
import asyncpg
import psycopg
from .config import settings


# Use settings from configuration
NEON_DB_URL = settings.neon_db_url
QDRANT_URL = settings.qdrant_url
QDRANT_API_KEY = settings.qdrant_api_key


class DatabaseManager:
    """Manages connections to both NeonDB (PostgreSQL) and Qdrant vector database."""

    def __init__(self):
        # Set up NeonDB (PostgreSQL) connection using SQLAlchemy
        self.neon_engine = create_engine(
            NEON_DB_URL,
            pool_pre_ping=True,  # Verify connections are alive before using them
            pool_recycle=300,    # Recycle connections every 5 minutes
            echo=False           # Set to True for SQL query logging
        )

        # Create session factory for NeonDB
        self.NeonDBSessionLocal = sessionmaker(
            autocommit=False,
            autoflush=False,
            bind=self.neon_engine
        )

        # Set up Qdrant client for vector database
        if QDRANT_API_KEY:
            self.qdrant_client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
                timeout=10
            )
        else:
            # For local development without API key
            self.qdrant_client = QdrantClient(host="localhost", port=6333)

        self.Base = declarative_base()

    def get_neon_db_session(self):
        """Get a NeonDB session for database operations."""
        session = self.NeonDBSessionLocal()
        try:
            yield session
        finally:
            session.close()

    def get_qdrant_client(self):
        """Get the Qdrant client for vector operations."""
        return self.qdrant_client

    def create_tables(self):
        """Create all database tables based on SQLAlchemy models."""
        self.Base.metadata.create_all(bind=self.neon_engine)

    async def test_connections(self) -> dict:
        """Test connections to both databases."""
        results = {
            "neon_db": False,
            "qdrant": False
        }

        # Test NeonDB connection
        try:
            with self.NeonDBSessionLocal() as session:
                # Try a simple query to test connection
                session.execute("SELECT 1")
                results["neon_db"] = True
        except Exception as e:
            print(f"NeonDB connection failed: {str(e)}")

        # Test Qdrant connection
        try:
            # Try to get collections to test connection
            collections = self.qdrant_client.get_collections()
            results["qdrant"] = True
        except Exception as e:
            print(f"Qdrant connection failed: {str(e)}")

        return results


# Create a global instance of the database manager
database_manager = DatabaseManager()


# Context manager for NeonDB sessions
@contextmanager
def get_db_session():
    """Context manager for NeonDB sessions."""
    db = database_manager.NeonDBSessionLocal()
    try:
        yield db
    finally:
        db.close()


# Initialize the database manager
def init_db():
    """Initialize database connections and create tables."""
    try:
        # Create all tables based on models
        database_manager.create_tables()
        print("Database tables created successfully")

        # Test the connections
        connection_status = asyncio.run(database_manager.test_connections())
        print(f"Database connection status: {connection_status}")

        return True
    except Exception as e:
        print(f"Error initializing database: {str(e)}")
        return False


if __name__ == "__main__":
    # Test the database connections when run as main
    import asyncio

    async def main():
        print("Testing database connections...")
        status = await database_manager.test_connections()
        print(f"Connection status: {status}")

        if status["neon_db"]:
            print("✓ NeonDB connection successful")
        else:
            print("✗ NeonDB connection failed")

        if status["qdrant"]:
            print("✓ Qdrant connection successful")
        else:
            print("✗ Qdrant connection failed")

    asyncio.run(main())