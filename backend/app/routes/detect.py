

from fastapi import APIRouter, File, UploadFile, Request, HTTPException, Depends
from datetime import datetime
import numpy as np
import cv2
import base64
import logging
from typing import List
import os 
from app.models.detection_result import DetectionResponse, Detection
from app.services.vision_utils import VisionPreprocessor
from app.utils.auth import verify_token

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/detect", response_model=DetectionResponse)
async def detect_intrusion(
    request: Request,
    file: UploadFile = File(...),
    user_id: str = Depends(verify_token)
):
   
    try:
        contents = await file.read()
        nparr = np.frombuffer(contents, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if image is None:
            raise HTTPException(status_code=400, detail="Invalid image format")
        
        # logger.info(f"Received image: {image.shape}")
        
        # Get services from app state
        yolo_detector = request.app.state.yolo_detector
        face_recognizer = request.app.state.face_recognizer
        firebase_service = request.app.state.firebase_service
        ws_manager = request.app.state.ws_manager
        
        # Check if YOLO detector is available
        if yolo_detector is None:
            # logger.error("YOLOv8 model not loaded!")
            raise HTTPException(
                status_code=503, 
                detail="YOLOv8 model not available."
            )
        
        # tang do tang phan cua anh neu anh duoc chup vao buoi toi 
        preprocessor = VisionPreprocessor()
        enhanced_image = preprocessor.enhance_for_night(image)
        
        person_detections = yolo_detector.detect_persons(enhanced_image)
        
        detections: List[Detection] = []
        alert_triggered = False
        
        
        for det in person_detections:
            bbox = det['bbox']
            confidence = det['confidence']
            
            # Extract person ROI for face recognition
            x1, y1, x2, y2 = map(int, bbox)
            
            # Validate bbox
            if x2 <= x1 or y2 <= y1:
                continue
                
            person_roi = enhanced_image[y1:y2, x1:x2]
            
            # # Run face recognition if available
            if face_recognizer is not None and person_roi.size > 0:        
                face_result = face_recognizer.recognize_face(person_roi)
                face_id = face_result.get('identity', 'unknow')
                is_known = face_result.get('is_known', False)
            else:
                
                face_id = 'unknown'
                is_known = False
                # logger.warning("Face recognition not available")
            
            # Trigger alert if unknown person detected
            if not is_known:
                alert_triggered = True
            
            detection = Detection(
                label="person",
                confidence=confidence,
                bbox=bbox,
                face_id=face_id,
                alert=not is_known
            )
            detections.append(detection)
            # Vẽ kết quả detection
            annotated_image = draw_detections(image, [det.dict() for det in detections])

        
        # Save to Firebase if available and detections found
        image_url = None
        timestamp = datetime.now().isoformat()
        
        if detections and firebase_service is not None:
            try:
                # Upload image to Firebase Storage
                image_filename = f"detections/{user_id}/{timestamp}.jpg"
                

                #  thu luu anh da duoc danh dau (annotation)

                _, buffer = cv2.imencode('.jpg', annotated_image)
                image_bytes = buffer.tobytes()
                
                image_url = firebase_service.upload_image(
                    image_bytes,
                    image_filename
                )
                
                # Save event to Firestore
                event_data = {
                    "user_id": user_id,
                    "timestamp": timestamp,
                    "detections": [det.dict() for det in detections],
                    "image_url": image_url,
                    "alert": alert_triggered
                }
                firebase_service.save_event(event_data)
                
                # logger.info(f"Event saved to Firebase: {len(detections)} detection(s)")
                
            except Exception as firebase_error:
                logger.error(f"Firebase error: {str(firebase_error)}")
                # logger.warning("Continuing without Firebase storage")
        else:
            # if firebase_service is None:
            #     logger.warning("Firebase not available, skipping storage")
            # Generate mock image URL for testing
            image_url = f"https://placeholder.example.com/detection_{timestamp}.jpg"
        
        # Broadcast to WebSocket clients
        if ws_manager and detections:
            try:
                await ws_manager.broadcast({
                    "type": "detection",
                    "data": {
                        "user_id": user_id,
                        "timestamp": timestamp,
                        "detections": [det.dict() for det in detections],
                        "image_url": image_url,
                        "alert": alert_triggered
                    }
                })
            except Exception as ws_error:
                logger.error(f"WebSocket broadcast error: {str(ws_error)}")
        response = DetectionResponse(
            detections=detections,
            image_url=image_url,
            timestamp=timestamp,
            alert=alert_triggered
        )
        
        # logger.info(f"Detection complete: {len(detections)} person(s) found, alert={alert_triggered}")
        return response
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Detection error: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500, 
            detail=f"Detection failed: {str(e)}"
        )
    

def draw_detections(image, detections):
    annotated = image.copy()
    for det in detections:
        x1, y1, x2, y2 = map(int, det['bbox'])
        label = f"{det.get('face_id', 'person')} ({det['confidence']:.2f})"
        color = (45, 255, 90) if not det.get('alert', False) else (0, 0, 255)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(annotated, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return annotated













