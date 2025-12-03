
import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)


class VisionPreprocessor:
    
    def __init__(self):
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    
    def enhance_for_night(self, image: np.ndarray) -> np.ndarray:
       
        # Check if image is too dark
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)
        
        if mean_brightness < 100:  # Dark image
            logger.info(f"Dark image detected (brightness: {mean_brightness:.2f}), applying enhancement")
            
            # Apply CLAHE to each channel
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            
            # Apply CLAHE to L channel
            l_enhanced = self.clahe.apply(l)
            
            # Merge and convert back
            lab_enhanced = cv2.merge([l_enhanced, a, b])
            enhanced = cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)
            
            # Denoise
            enhanced = cv2.fastNlMeansDenoisingColored(enhanced, None, 10, 10, 7, 21)
            
            # Gamma correction
            enhanced = self.adjust_gamma(enhanced, gamma=1.5)
            
            return enhanced
        else:
            # Image is bright enough, minimal processing
            return self.denoise(image)
    
    def denoise(self, image: np.ndarray) -> np.ndarray:
        return cv2.fastNlMeansDenoisingColored(image, None, 5, 5, 7, 21)
    
    def adjust_gamma(self, image: np.ndarray, gamma: float = 1.0) -> np.ndarray:
        
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 
                         for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)
    
    def apply_clahe(self, image: np.ndarray) -> np.ndarray:
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    
    def sharpen(self, image: np.ndarray) -> np.ndarray:
        kernel = np.array([[-1,-1,-1],
                          [-1, 9,-1],
                          [-1,-1,-1]])
        return cv2.filter2D(image, -1, kernel)