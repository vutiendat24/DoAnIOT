
from fastapi import APIRouter, Request, HTTPException, Depends
from pydantic import BaseModel
from datetime import datetime
import logging

from app.utils.auth import verify_token

logger = logging.getLogger(__name__)
router = APIRouter()


class DoorCommandResponse(BaseModel):
    success: bool
    message: str
    command_id: str
    timestamp: str


@router.post("/door/open", response_model=DoorCommandResponse)
async def open_door(
    request: Request,
    user_id: str = Depends(verify_token)
):
    """
    Gửi lệnh mở cửa lên Firebase Realtime Database.
    ESP32 sẽ lắng nghe real-time và thực hiện mở cửa ngay lập tức.
    """
    try:
        firebase_service = request.app.state.firebase_service
        
        if firebase_service is None:
            raise HTTPException(
                status_code=503,
                detail="Firebase service not available"
            )
        
        timestamp = datetime.now().isoformat()
        
        # Gửi lệnh mở cửa lên Firebase (Firestore + Realtime Database)
        command_id = firebase_service.send_door_command(
            user_id=user_id,
            action="open",
            timestamp=timestamp
        )
        
        # Broadcast qua WebSocket để cập nhật UI
        ws_manager = request.app.state.ws_manager
        if ws_manager:
            try:
                await ws_manager.broadcast({
                    "type": "door_command",
                    "data": {
                        "user_id": user_id,
                        "action": "open",
                        "command_id": command_id,
                        "timestamp": timestamp
                    }
                })
            except Exception as ws_error:
                logger.error(f"WebSocket broadcast error: {str(ws_error)}")
        
        logger.info(f"Door open command sent: {command_id}")
        
        return DoorCommandResponse(
            success=True,
            message="Lệnh mở cửa đã được gửi thành công!",
            command_id=command_id,
            timestamp=timestamp
        )
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Door command error: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Không thể gửi lệnh mở cửa: {str(e)}"
        )
