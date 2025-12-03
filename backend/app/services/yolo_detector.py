

from ultralytics import YOLO
import numpy as np
import logging
from typing import List, Dict

logger = logging.getLogger(__name__)


class YoloDetector:
    
    def __init__(self, model_path: str):
       
        self.model = YOLO(model_path)
        self.person_class_id = 0  
        logger.info(f"YOLOv8 model loaded from {model_path}")
    
    def detect_persons(
        self,
        image: np.ndarray,
        confidence_threshold: float = 0.5
    ) -> List[Dict]:
      
        results = self.model(image, verbose=False)
        
        detections = []
        
        for result in results:
            boxes = result.boxes
            
            for box in boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                
                # Filter for person class only
                if class_id == self.person_class_id and confidence >= confidence_threshold:
                    bbox = box.xyxy[0].cpu().numpy().tolist()
                    
                    detections.append({
                        'bbox': bbox,
                        'confidence': confidence,
                        'class_id': class_id
                    })
        
        logger.info(f"Detected {len(detections)} person(s)")
        return detections
    
    def detect_with_roi(
        self,
        image: np.ndarray,
        roi: Dict,
        confidence_threshold: float = 0.5
    ) -> List[Dict]:
        
        all_detections = self.detect_persons(image, confidence_threshold)
        
        roi_x, roi_y = roi['x'], roi['y']
        roi_w, roi_h = roi['width'], roi['height']
        
        filtered_detections = []
        
        for det in all_detections:
            x1, y1, x2, y2 = det['bbox']
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            if (roi_x <= center_x <= roi_x + roi_w and
                roi_y <= center_y <= roi_y + roi_h):
                filtered_detections.append(det)
        
        return filtered_detections