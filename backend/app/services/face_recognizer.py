
import torch
import torch.nn as nn
import cv2

import os
import numpy as np
from typing import Dict, List, Optional
import logging
from torchvision.models import resnet50
logger = logging.getLogger(__name__)


class ArcFaceModel(nn.Module):
    
    def __init__(self, embedding_size=128, num_classes=2):
        super(ArcFaceModel, self).__init__()
        self.backbone = resnet50(weights='DEFAULT')
        self.backbone.fc = nn.Linear(self.backbone.fc.in_features, embedding_size)
        self.backbone_bn = nn.BatchNorm1d(embedding_size)
        self.backbone_bn.bias.requires_grad_(False)

    def forward(self, x):
        x = self.backbone(x)
        x = self.backbone_bn(x)
        return x


class FaceRecognizer:
    
    def __init__(self, model_path: str, embedding_size: int = 128):        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.embedding_size = embedding_size
        
        self.model = ArcFaceModel(embedding_size)
        
        try:
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            logger.info(f"ArcFace model loaded from {model_path}")
        except FileNotFoundError:
            logger.warning(f"Model file {model_path} not found, using untrained model")
        
        self.model.to(self.device)
        self.model.eval()
        
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        
        self.whitelist: Dict[str, np.ndarray] = {}
        
        logger.info("FaceRecognizer initialized")
    
    def detect_faces(self, image: np.ndarray) -> List[tuple]:
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        return faces
    
    def extract_embedding(self, face_image: np.ndarray) -> np.ndarray:
      
        # Preprocess
        face_resized = cv2.resize(face_image, (224, 224))
        face_rgb = cv2.cvtColor(face_resized, cv2.COLOR_BGR2RGB)
        face_tensor = torch.from_numpy(face_rgb).permute(2, 0, 1).float() / 255.0
        face_tensor = face_tensor.unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            embedding = self.model(face_tensor)
            embedding = embedding.cpu().numpy().flatten()
        
        embedding = embedding / np.linalg.norm(embedding)
        
        return embedding
    
    def recognize_face(
        self,
        person_image: np.ndarray,
        threshold: float = 0.6
    ) -> Dict:
       
        faces = self.detect_faces(person_image)
        save_faces(person_image, faces)
        if len(faces) == 0:
            return {
                'identity': 'no_face',
                'is_known': False,
                'confidence': 0.0
            }
        
        faces_sorted = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)
        x, y, w, h = faces_sorted[0]
        
        face_roi = person_image[y:y+h, x:x+w]
        
        embedding = self.extract_embedding(face_roi)
        
        best_match = None
        best_similarity = 0.0
        
        for identity, whitelist_embedding in self.whitelist.items():
            similarity = np.dot(embedding, whitelist_embedding)
            
            if similarity > best_similarity:
                best_similarity = similarity
                best_match = identity
        
        is_known = best_similarity >= threshold if best_match else False
        
        return {
            'identity': best_match if is_known else 'unknownk',
            'is_known': is_known,
            'confidence': float(best_similarity)
        }
    
    def load_whitelist_from_firebase(self, firebase_service):
       
        try:
            whitelist_data = firebase_service.get_whitelist()
            
            for entry in whitelist_data:
                identity = entry['identity']
                embedding = np.array(entry['embedding'])
                self.whitelist[identity] = embedding
            
            logger.info(f"Loaded {len(self.whitelist)} whitelist entries")
        except Exception as e:
            logger.error(f"Failed to load whitelist: {str(e)}")
    
    def add_to_whitelist(self, identity: str, embedding: np.ndarray):
        self.whitelist[identity] = embedding
        logger.info(f"Added {identity} to whitelist")

import os
from datetime import datetime

def save_faces(
    image: np.ndarray,
    faces,
    save_dir="saved_faces"
):
    os.makedirs(save_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    for i, (x, y, w, h) in enumerate(faces):
        face_roi = image[y:y+h, x:x+w]

        if face_roi.size == 0:
            continue

        filename = f"face_{timestamp}_{i}.jpg"
        cv2.imwrite(os.path.join(save_dir, filename), face_roi)



