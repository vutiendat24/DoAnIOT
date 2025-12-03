
from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    """Application settings"""

    # Firebase
    FIREBASE_CREDENTIALS: str = "serviceAccountKey.json"
    FIREBASE_STORAGE_BUCKET: str = "thef.appspot.com"
    PROJECT_ID: str = "thef-detect"
   
    # AI Models
    YOLO_MODEL_PATH: str = "models/yolov8n.pt"
    ARCFACE_MODEL_PATH: str = "models/best_arcface_model.pth"
    
    # Server
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    DEBUG: bool = False
    
    # Detection
    CONFIDENCE_THRESHOLD: float = 0.5
    FACE_RECOGNITION_THRESHOLD: float = 0.6
    
    class Config:
        env_file = ".env"
        extra = "ignore"
        case_sensitive = True


# Global settings instance
settings = Settings()
print(">>> [DEBUG CONFIG PATHHHHHHHHH] ARCFACE_MODEL_PATH =", settings.ARCFACE_MODEL_PATH)
