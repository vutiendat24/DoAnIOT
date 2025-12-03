

from fastapi import APIRouter, File, UploadFile, Request, HTTPException, Depends
from datetime import datetime
import numpy as np
from time import time

import cv2
import base64
import logging
from typing import List, Optional
import asyncio
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
        begin = time()  # ---- START ----

        # Read file
        contents = await file.read()

        # Decode image
        nparr = np.frombuffer(contents, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        time_count_image = time()

        if image is None:
            raise HTTPException(status_code=400, detail="Invalid image format")
        
        # Setup service
        time_count_setup_begin = time()
        yolo_detector = request.app.state.yolo_detector
        face_recognizer = request.app.state.face_recognizer
        firebase_service = request.app.state.firebase_service
        ws_manager = request.app.state.ws_manager
        time_count_setup_end = time()
        
        if yolo_detector is None:
            raise HTTPException(
                status_code=503, 
                detail="YOLOv8 model not available."
            )
        
        # Enhance image
        time_enhance_begin = time()
        preprocessor = VisionPreprocessor()
        enhanced_image = preprocessor.enhance_for_night(image)
        time_enhance_end = time()
        # --- YOLO detection ---
        time_before_yolo = time()
        person_detections = yolo_detector.detect_persons(enhanced_image)
        time_after_yolo = time()

        detections: List[Detection] = []
        alert_triggered = False
        
        # --- Face recognition ---
        time_before_face = time()
        for det in person_detections:
            bbox = det['bbox']
            confidence = det['confidence']
            
            x1, y1, x2, y2 = map(int, bbox)
            if x2 <= x1 or y2 <= y1:
                continue
                
            person_roi = enhanced_image[y1:y2, x1:x2]
            
            if face_recognizer is not None and person_roi.size > 0:
                face_result = face_recognizer.recognize_face(person_roi)
                face_id = face_result.get('identity', 'unknown')
                is_known = face_result.get('is_known', False)
            else:
                face_id = 'unknown'
                is_known = False
            
            if not is_known:
                alert_triggered = True
            
            detections.append(Detection(
                label="person",
                confidence=confidence,
                bbox=bbox,
                face_id=face_id,
                alert=not is_known
            ))
        
        time_after_face = time()

        # Drawing
        annotated_image = draw_detections(image, [det.dict() for det in detections])

        # Firebase upload (offloaded to background to reduce API latency)
        image_url: Optional[str] = None
        timestamp = datetime.now().isoformat()

        time_before_firebase = time()

        async def _upload_and_save(image_to_upload: bytes, filename: str, event_payload: dict):
            try:
                # Run blocking uploads in threadpool
                url = await asyncio.to_thread(firebase_service.upload_image, image_to_upload, filename)
                event_payload['image_url'] = url
                await asyncio.to_thread(firebase_service.save_event, event_payload)
                logger.info(f"Background Firebase work completed: {filename}")
            except Exception as e:
                logger.error(f"Background Firebase error: {str(e)}")

        if detections and firebase_service is not None:
            try:
                image_filename = f"detections/{user_id}/{timestamp}.jpg"

                # Compress/rescale image before upload to reduce size (speeds up network transfer)
                # keep aspect ratio, limit max dimension to 1280
                h, w = annotated_image.shape[:2]
                max_dim = 1280
                if max(h, w) > max_dim:
                    scale = max_dim / max(h, w)
                    new_w = int(w * scale)
                    new_h = int(h * scale)
                    upload_image = cv2.resize(annotated_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
                else:
                    upload_image = annotated_image

                # Encode with reasonable JPEG quality to shrink size
                encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                _, buffer = cv2.imencode('.jpg', upload_image, encode_params)
                image_bytes = buffer.tobytes()

                event_data = {
                    "user_id": user_id,
                    "timestamp": timestamp,
                    "detections": [det.dict() for det in detections],
                    "image_url": None,
                    "alert": alert_triggered
                }

                # Fire-and-forget background upload/save so API response is fast.
                # We still return a placeholder URL so client has a value quickly.
                asyncio.create_task(_upload_and_save(image_bytes, image_filename, event_data))
                image_url = f"https://placeholder.example.com/detection_{timestamp}.jpg"

            except Exception as firebase_error:
                logger.error(f"Firebase error (scheduling): {str(firebase_error)}")
                image_url = f"https://placeholder.example.com/detection_{timestamp}.jpg"
        else:
            image_url = f"https://placeholder.example.com/detection_{timestamp}.jpg"

        time_after_firebase = time()
        time_web_socket_begin = time()
        # --- WebSocket Broadcast ---
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
        time_web_socket_end = time()

        # ---- LOG PERFORMANCE ----
        time_end = time()
        logger.info({
            "time_total": time_end - begin,
            "time_decode_image": time_count_image - begin,
            "time_service_setup": time_count_setup_end - time_count_setup_begin,
            "time_enhance": time_enhance_end - time_enhance_begin,
            "time_yolo": time_after_yolo - time_before_yolo,
            "time_face_recognition": time_after_face - time_before_face,
            "time_firebase": time_after_firebase - time_before_firebase if detections else 0,
            "time_web_socket" : time_web_socket_end - time_web_socket_begin
        })

        return DetectionResponse(
            detections=detections,
            image_url=image_url,
            timestamp=timestamp,
            alert=alert_triggered
        )
        
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













