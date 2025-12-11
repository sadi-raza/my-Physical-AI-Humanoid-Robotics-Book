"""Authentication API endpoints using Better-Auth for user management."""

from fastapi import APIRouter, Depends, HTTPException, status
from typing import Optional
import uuid
from datetime import datetime, timedelta
from pydantic import BaseModel
from ..models.user import User, UserCreate, UserUpdate

router = APIRouter(prefix="/auth", tags=["authentication"])


class LoginRequest(BaseModel):
    email: str
    password: str


class LoginResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: User


class TokenData(BaseModel):
    user_id: str
    email: str


# Mock database for demonstration purposes
# In a real implementation, this would connect to NeonDB
mock_users_db = {}


@router.post("/register", response_model=User)
async def register_user(user_create: UserCreate):
    """
    Register a new user with Better-Auth integration.
    Stores user background information for personalization as required by FR-006.
    """
    # Check if user already exists
    for existing_user in mock_users_db.values():
        if existing_user.email == user_create.email:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

    # Create new user
    user_id = str(uuid.uuid4())
    now = datetime.utcnow()

    # In a real implementation, the password would be hashed
    new_user = User(
        id=user_id,
        email=user_create.email,
        name=user_create.name,
        background=user_create.background,
        research_interests=user_create.research_interests,
        created_at=now,
        updated_at=now
    )

    mock_users_db[user_id] = new_user

    return new_user


@router.post("/login", response_model=LoginResponse)
async def login_user(login_request: LoginRequest):
    """
    Authenticate user and return access token.
    Implements Better-Auth integration as specified in the constitution.
    """
    # Find user by email
    user = None
    for u in mock_users_db.values():
        if u.email == login_request.email:
            user = u
            break

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # In a real implementation, verify hashed password here
    # For this mock, we'll just check if the password is not empty
    if not login_request.password:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )

    # Generate a mock token (in real implementation, use JWT)
    access_token = f"mock_token_{user.id}_{int(datetime.now().timestamp())}"

    return LoginResponse(
        access_token=access_token,
        token_type="bearer",
        user=user
    )


@router.get("/me", response_model=User)
async def get_current_user(token: str = None):
    """
    Get current authenticated user information.
    Used for personalization as the system needs to store user background information.
    """
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )

    # Extract user ID from token (mock implementation)
    try:
        # In a real implementation, this would decode the JWT token
        # For this mock, we'll extract user ID from the mock token format
        if not token.startswith("mock_token_"):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )

        user_id = token.split("_")[2]
        user = mock_users_db.get(user_id)

        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found"
            )

        return user
    except Exception:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )


@router.put("/profile", response_model=User)
async def update_profile(user_update: UserUpdate, token: str = None):
    """
    Update user profile information.
    Enables personalization by allowing users to update their background information.
    """
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )

    try:
        # Extract user ID from token (mock implementation)
        if not token.startswith("mock_token_"):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )

        user_id = token.split("_")[2]
        user = mock_users_db.get(user_id)

        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found"
            )

        # Update user information based on provided fields
        if user_update.name is not None:
            user.name = user_update.name
        if user_update.background is not None:
            user.background = user_update.background
        if user_update.research_interests is not None:
            user.research_interests = user_update.research_interests

        user.updated_at = datetime.utcnow()

        # Update in mock database
        mock_users_db[user_id] = user

        return user
    except Exception:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )


@router.delete("/logout")
async def logout_user(token: str = None):
    """
    Logout user (invalidate token).
    """
    # In a real implementation, this would invalidate the token
    # For this mock implementation, we'll just return success
    return {"message": "Successfully logged out"}