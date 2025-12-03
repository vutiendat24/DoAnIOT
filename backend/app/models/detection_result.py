
import datetime
from pydantic import BaseModel
from typing import List, Optional


class Detection(BaseModel):
    label: str
    confidence: float
    bbox: List[float]  # [x1, y1, x2, y2] nay dung de luu vi tri cua nguoi 
    face_id: str
    alert: bool


class DetectionResponse(BaseModel):
    detections: List[Detection]
    image_url: Optional[str] = None
    timestamp: str
    alert: bool = False