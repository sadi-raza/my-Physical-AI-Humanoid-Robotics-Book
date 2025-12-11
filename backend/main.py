"""Main application entry point for the Physical AI & Humanoid Robotics Textbook Platform."""

import uvicorn
from src.config import settings
from src.api.main import app


if __name__ == "__main__":
    print(f"Starting {settings.app_name} v{settings.app_version} on {settings.host}:{settings.port}")
    print(f"Debug mode: {settings.debug}")
    print(f"Reload mode: {settings.debug}")

    uvicorn.run(
        "src.api.main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug,
        log_level=settings.log_level.lower()
    )