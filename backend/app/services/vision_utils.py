import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)

class VisionPreprocessor:
    
    def __init__(self):
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    
    def enhance_for_night(self, image: np.ndarray) -> np.ndarray:
        h, w = image.shape[:2]
        if max(h, w) > 1280:
            scale = 1280 / max(h, w)
            new_w = int(w * scale)
            new_h = int(h * scale)
            image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # 2. Kiểm tra độ sáng
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)
        
        # Ngưỡng: < 80 là tối 
        if mean_brightness < 80:  
            print("ap dung them tang cuong sang")
            # --- Bước 1: CLAHE (Cân bằng sáng cục bộ) ---
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l_enhanced = self.clahe.apply(l)
            lab_enhanced = cv2.merge([l_enhanced, a, b])
            enhanced = cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)
            
            # --- Bước 2: Gamma Correction (Tăng độ sáng tổng thể) ---
            enhanced = self.adjust_gamma(enhanced, gamma=1.3)
            
            # --- Bước 3: Denoise ---
            # Chỉ dùng Blur nhẹ để giảm nhiễu muối tiêu
            enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)
            
            return enhanced
        else:
            # Nếu ảnh đủ sáng, trả về nguyên bản ngay lập tức

            return image
    
    def adjust_gamma(self, image: np.ndarray, gamma: float = 1.0) -> np.ndarray:
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 
                          for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)
    
    def denoise(self, image: np.ndarray) -> np.ndarray:
        # Cảnh báo: Hàm này rất chậm
        return cv2.GaussianBlur(image, (5, 5), 0) 