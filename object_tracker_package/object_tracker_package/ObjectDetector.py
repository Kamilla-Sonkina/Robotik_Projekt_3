import cv2
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self, model_path, device):
        self.device = device
        self.model = YOLO(model_path).to(device)
      
    def detect_objects(self, frame):
        results = self.model(frame)
        if results and results[0].boxes:
            return results[0].boxes.data.tolist()
        else:
            return []

    def draw_boxes(self, frame, results):
        
        for result in results:
            if len(result) >= 6:
                x1, y1, x2, y2 = [int(coord * 2) for coord in result[:4]]
               
                conf = result[4]
                cls = int(result[5])
                label = 'Einhorn' if cls == 1 else 'Katze'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
      
   
