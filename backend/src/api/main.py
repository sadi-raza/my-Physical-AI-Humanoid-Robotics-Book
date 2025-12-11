"""Main FastAPI application entry point for the Physical AI & Humanoid Robotics Textbook Platform."""

from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from .auth import router as auth_router
from .rag import router as rag_router
from .personalize import router as personalize_router
from .translate_ur import router as translate_ur_router
from ..utils.content_validator import ContentValidator
from datetime import datetime
import os
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook Platform API",
    description="REST API for the Physical AI & Humanoid Robotics textbook platform with RAG, personalization, and translation capabilities",
    version="1.0.0",
    contact={
        "name": "Physical AI & Humanoid Robotics Team",
        "url": "https://github.com/Q4AIAgents/my-Physical-AI-Humanoid-Robotics-Book",
    },
    license_info={
        "name": "MIT License",
        "url": "https://opensource.org/licenses/MIT",
    },
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(auth_router)
app.include_router(rag_router)
app.include_router(personalize_router)
app.include_router(translate_ur_router)

# Mount static files if needed
# app.mount("/static", StaticFiles(directory="static"), name="static")


@app.get("/")
async def root():
    """Root endpoint for the API."""
    return {
        "message": "Welcome to the Physical AI & Humanoid Robotics Textbook Platform API",
        "description": "This API provides access to textbook content, RAG functionality, personalization, and translation services",
        "endpoints": {
            "auth": "/auth/",
            "rag": "/rag/",
            "personalize": "/api/personalize",
            "translate_ur": "/api/translate-ur"
        },
        "timestamp": datetime.utcnow()
    }


@app.get("/health")
async def health_check():
    """Health check endpoint for the entire application."""
    return {
        "status": "healthy",
        "service": "Physical AI & Humanoid Robotics Textbook Platform API",
        "version": "1.0.0",
        "timestamp": datetime.utcnow(),
        "features": {
            "authentication": "active",
            "rag_service": "active",
            "personalization": "active",
            "translation_urdu": "active"
        }
    }


@app.get("/api/content/{module}/{chapter}")
async def get_content(module: str, chapter: str):
    """
    Retrieve textbook content for a specific module and chapter.
    This endpoint provides access to the structured textbook content.
    """
    # In a real implementation, this would retrieve content from the database
    # For this mock implementation, we'll return a sample response

    content_sample = {
        "module": module,
        "chapter": chapter,
        "title": f"{module.title()} - {chapter.replace('-', ' ').title()}",
        "content": f"This is sample content for {module}/{chapter}. In a real implementation, this would retrieve actual textbook content from the database.",
        "outcomes": ["Learning outcome 1", "Learning outcome 2"],
        "exercises": [{"question": "Sample exercise question?", "answer": "Sample answer"}],
        "code_examples": [{"language": "python", "code": "# Sample code example"}],
        "diagrams": [{"type": "mermaid", "content": "graph TD; A-->B; B-->C;"}],
        "created_at": datetime.utcnow(),
        "updated_at": datetime.utcnow()
    }

    return content_sample


@app.get("/api/modules")
async def get_modules():
    """
    Retrieve list of available modules.
    Returns the 5 core modules (ROS2, Gazebo/Unity, Isaac, VLA, Capstone) as specified in the requirements.
    """
    modules = [
        {"id": "ros2", "name": "ROS2", "title": "Robot Operating System 2", "description": "Fundamentals and advanced concepts of ROS2"},
        {"id": "gazebo-unity", "name": "Gazebo/Unity", "title": "Simulation Environments", "description": "Gazebo and Unity for robotics simulation"},
        {"id": "isaac", "name": "Isaac", "title": "NVIDIA Isaac Platform", "description": "NVIDIA Isaac robotics platform and tools"},
        {"id": "vla", "name": "VLA", "title": "Vision-Language-Action", "description": "Vision-Language-Action models for robotic manipulation"},
        {"id": "capstone", "name": "Capstone", "title": "Integration and Applications", "description": "Integration of all concepts and advanced applications"}
    ]

    return {"modules": modules}


@app.get("/api/validate-content-quality")
async def validate_content_quality(content: str):
    """
    Validate that content meets grade 10-12 level and quality requirements.
    Implements the content validation required by FR-008.
    """
    validator = ContentValidator()
    validation_results = validator.validate_content_quality(content)

    return {
        "content_valid": validation_results["is_valid"],
        "validation_details": validation_results,
        "timestamp": datetime.utcnow()
    }


@app.get("/api/config")
async def get_config():
    """Get application configuration and requirements compliance status."""
    return {
        "configuration": {
            "project_focus": "Physical AI & Humanoid Robotics",
            "modules": ["ROS2", "Gazebo/Unity", "Isaac", "VLA", "Capstone"],
            "rag_stack": ["FastAPI", "Qdrant", "NeonDB", "ChatKit", "OpenAI Agents"],
            "deployment_platforms": {
                "frontend": ["GitHub Pages", "Vercel"],
                "backend": ["Railway", "Render", "Fly.io"],
                "vectors": ["Qdrant Cloud"],
                "relational": ["NeonDB"]
            }
        },
        "compliance_status": {
            "quality_first_non_negotiable": True,
            "rag_architecture_standard": True,
            "safety_first_design": True,
            "multi_language_support": True,
            "modular_subagents": True
        },
        "performance_goals": {
            "rag_response_time": "<800ms",
            "concurrent_users": 1000,
            "vector_embeddings": "1M+ in Qdrant"
        }
    }


# Event handlers
@app.on_event("startup")
async def startup_event():
    """Startup event handler."""
    logger.info("Starting Physical AI & Humanoid Robotics Textbook Platform API")
    # Initialize services, database connections, etc.


@app.on_event("shutdown")
async def shutdown_event():
    """Shutdown event handler."""
    logger.info("Shutting down Physical AI & Humanoid Robotics Textbook Platform API")
    # Clean up resources, close connections, etc.


# Error handlers
@app.exception_handler(404)
async def not_found_handler(request, exc):
    """Handle 404 errors."""
    return {
        "error": "Endpoint not found",
        "path": str(request.url),
        "timestamp": datetime.utcnow()
    }


@app.exception_handler(500)
async def internal_error_handler(request, exc):
    """Handle 500 errors."""
    return {
        "error": "Internal server error",
        "detail": str(exc),
        "timestamp": datetime.utcnow()
    }