import cv2
import requests
import time
from datetime import datetime
LOCAL_IP= "192.168.1.136"
API_URL = f'http://{LOCAL_IP}:8000/api/detect' 
TOKEN = "test_token"                
INTERVAL = 10                          

cap = cv2.VideoCapture(0) 

if not cap.isOpened():
    print("Không mở được camera laptop!")
    exit()

print("Bắt đầu gửi ảnh lên server ...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Không thể đọc khung hình từ camera")
        break

    
    _, img_encoded = cv2.imencode(".jpg", frame)
    img_bytes = img_encoded.tobytes()

    # Tạo payload multipart/form-data
    files = {
        "file": ("image.jpg", img_bytes, "image/jpeg")
    }

    headers = {
        "Authorization": f"Bearer {TOKEN}"
    }

    try:
        response = requests.post(API_URL, files=files, headers=headers, timeout=5)
        print(f"[{datetime.now()}] Server trả về:", response.json())
    except Exception as e:
        print("Lỗi gửi ảnh:", e)

    time.sleep(INTERVAL) 
