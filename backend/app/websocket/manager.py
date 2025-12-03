

from fastapi import WebSocket
from typing import List
import json
import logging
import asyncio

logger = logging.getLogger(__name__)


class ConnectionManager:
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"Client connected. Total connections: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"Client disconnected. Total connections: {len(self.active_connections)}")
    
    async def send_personal_message(self, message: dict, websocket: WebSocket):
        await websocket.send_text(json.dumps(message))
    
    async def broadcast(self, message: dict):
        # Send to all clients concurrently to reduce latency
        coros = []
        for connection in list(self.active_connections):
            coros.append(self._safe_send(connection, message))

        await asyncio.gather(*coros, return_exceptions=True)

    async def _safe_send(self, connection, message: dict):
        try:
            await connection.send_text(json.dumps(message))
        except Exception as e:
            logger.error(f"Failed to send to client: {str(e)}")
            # remove disconnected connection
            try:
                self.disconnect(connection)
            except Exception:
                pass