
import firebase_admin
from firebase_admin import credentials, firestore, storage, auth, db as realtime_db
from typing import Dict, List, Optional
import logging
from datetime import datetime
import io
import cloudinary
import cloudinary.uploader
import logging
from dotenv import load_dotenv
import os

load_dotenv()
logger = logging.getLogger(__name__)


class FirebaseService:
    
    def __init__(self, credentials_path: str):
        
        try:
            cred = credentials.Certificate(credentials_path)
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'storageBucket': 'thef-detect.appspot.com',
                    'databaseURL': 'https://thef-detect-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            
            self.db = firestore.client()
            self.bucket = storage.bucket()
            self.rtdb = realtime_db  # Firebase Realtime Database
            cloudinary.config(
                cloud_name=os.getenv("CLOUD_NAME"),
                api_key=os.getenv("CLOUD_API_KEY"),
                api_secret=os.getenv("CLOUD_API_SECRET")
            )
            logger.info("Firebase initialized successfully (Firestore + Realtime Database)")
        except Exception as e:
            logger.error(f"Firebase initialization failed: {str(e)}")
            raise
    
    def upload_image(self, image_bytes: bytes, filename: str) -> str:
    
        try:
            result = cloudinary.uploader.upload(
                image_bytes,
                public_id=filename.split('.')[0],
                resource_type="image"
            )
            url = result["secure_url"]
            # logger.info(f"Image uploaded to Cloudinary: {url}")
            return url
        except Exception as e:
            logger.error(f"Cloudinary upload failed: {str(e)}")
            raise
    
    def save_event(self, event_data: Dict) -> str:
       
        try:
            doc_ref = self.db.collection('events').document()
            event_data['created_at'] = firestore.SERVER_TIMESTAMP
            doc_ref.set(event_data)
            
            # logger.info(f"Event saved: {doc_ref.id}")
            return doc_ref.id
        except Exception as e:
            logger.error(f"Event save failed: {str(e)}")
            raise
    
    def get_events(
        self,
        user_id: str,
        limit: int = 50, 
        alert_only: bool = False
    ) -> List[Dict]:
       
        try:
            query = self.db.collection('events').where('user_id', '==', user_id)
            
            # if alert_only:
            #     query = query.where('alert', '==', True)

            
            query = query.order_by('created_at', direction=firestore.Query.DESCENDING).limit(limit)
            
            events = []
            for doc in query.stream():
                event = doc.to_dict()
                event['id'] = doc.id
                events.append(event)
            
            return events
        except Exception as e:
            logger.error(f"Event retrieval failed: {str(e)}")
            return []
    
    def get_event_by_id(self, event_id: str, user_id: str) -> Optional[Dict]:
        
        try:
            doc = self.db.collection('events').document(event_id).get()
            
            if doc.exists:
                event = doc.to_dict()
                
                # Verify user ownership
                if event.get('user_id') == user_id:
                    event['id'] = doc.id
                    return event
            
            return None
        except Exception as e:
            logger.error(f"Event retrieval failed: {str(e)}")
            return None
    
    def save_roi(self, roi_data: Dict) -> str:
    
        try:
            doc_ref = self.db.collection('rois').document()
            roi_data['created_at'] = firestore.SERVER_TIMESTAMP
            doc_ref.set(roi_data)
            
            logger.info(f"ROI saved: {doc_ref.id}")
            return doc_ref.id
        except Exception as e:
            logger.error(f"ROI save failed: {str(e)}")
            raise
    
    def get_user_rois(self, user_id: str) -> List[Dict]:
        try:
            query = self.db.collection('rois').where('user_id', '==', user_id)
            
            rois = []
            for doc in query.stream():
                roi = doc.to_dict()
                roi['id'] = doc.id
                rois.append(roi)
            
            return rois
        except Exception as e:
            logger.error(f"ROI retrieval failed: {str(e)}")
            return []
    
    def delete_roi(self, roi_id: str, user_id: str):
        try:
            doc_ref = self.db.collection('rois').document(roi_id)
            doc = doc_ref.get()
            
            if doc.exists and doc.to_dict().get('user_id') == user_id:
                doc_ref.delete()
                logger.info(f"ROI deleted: {roi_id}")
            else:
                raise ValueError("ROI not found or unauthorized")
        except Exception as e:
            logger.error(f"ROI deletion failed: {str(e)}")
            raise
    
    def verify_token(self, id_token: str) -> Dict:
        
        try:
            decoded_token = auth.verify_id_token(id_token)
            return decoded_token
        except Exception as e:
            logger.error(f"Token verification failed: {str(e)}")
            raise
    
    def get_whitelist(self) -> List[Dict]:
        """Get face recognition whitelist"""
        try:
            query = self.db.collection('whitelist')
            
            whitelist = []
            for doc in query.stream():
                entry = doc.to_dict()
                entry['id'] = doc.id
                whitelist.append(entry)
            
            return whitelist
        except Exception as e:
            logger.error(f"Whitelist retrieval failed: {str(e)}")
            return []
    
    def add_to_whitelist(self, identity: str, embedding: List[float]) -> str:
        try:
            doc_ref = self.db.collection('whitelist').document()
            doc_ref.set({
                'identity': identity,
                'embedding': embedding,
                'created_at': firestore.SERVER_TIMESTAMP
            })
            
            logger.info(f"Added to whitelist: {identity}")
            return doc_ref.id
        except Exception as e:
            logger.error(f"Whitelist addition failed: {str(e)}")
            raise
    
    def send_door_command(self, user_id: str, action: str, timestamp: str) -> str:
        """
        Gửi lệnh điều khiển cửa lên Firebase.
        - Ghi vào Firestore để lưu lịch sử
        - Ghi vào Realtime Database để ESP32 lắng nghe real-time
        """
        try:
            # 1. Lưu vào Firestore (lịch sử)
            doc_ref = self.db.collection('door_commands').document()
            command_data = {
                'user_id': user_id,
                'action': action,
                'timestamp': timestamp,
                'executed': False,
                'created_at': firestore.SERVER_TIMESTAMP
            }
            doc_ref.set(command_data)
            command_id = doc_ref.id
            
            # 2. Ghi vào Realtime Database để ESP32 lắng nghe real-time
            rtdb_data = {
                'command_id': command_id,
                'user_id': user_id,
                'action': action,
                'timestamp': timestamp,
                'executed': False
            }
            self.rtdb.reference('door_command').set(rtdb_data)
            
            logger.info(f"Door command sent to Firestore + RTDB: {command_id} - {action}")
            return command_id
        except Exception as e:
            logger.error(f"Door command failed: {str(e)}")
            raise