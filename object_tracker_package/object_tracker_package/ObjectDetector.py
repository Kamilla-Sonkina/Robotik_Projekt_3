import cv2
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self, model_path, device, confidence_threshold=0.60): # Confidence desmodels aus F1-Graph 
        """
        Initialisiert den Objektdetektor.

        Args:
            model_path (str): Pfad zum YOLO-Modell.
            device (str): Gerät, auf dem das Modell ausgeführt wird (z.B. 'cpu' oder 'cuda').
            confidence_threshold (float): Konfidenzschwelle für die Objekterkennung.
        """
        self.device = device
        self.model = YOLO(model_path).to(device)
        self.confidence_threshold = confidence_threshold
      
    def detect_objects(self, frame):
        """
        Erkenne Objekte in einem Frame.

        Args:
            frame (numpy.ndarray): Das Eingabeframe.

        Returns:
            list: Liste erkannter Objekte mit ihren Begrenzungsrahmen und Klassifizierungen.
        """
        results = self.model(frame)
        if results and results[0].boxes:
            filtered_boxes = [box for box in results[0].boxes.data.tolist() if box[4] >= self.confidence_threshold]
            return filtered_boxes
        else:
            return []

    def draw_boxes(self, frame, results):
        """
        Zeichne Begrenzungsrahmen um erkannte Objekte im Frame.

        Args:
            frame (numpy.ndarray): Das Eingabeframe.
            results (list): Liste erkannter Objekte mit ihren Begrenzungsrahmen und Klassifizierungen.
        """
        for result in results:
            if len(result) >= 6:
                x1, y1, x2, y2 = [int(coord * 2) for coord in result[:4]]
                conf = result[4]
                cls = int(result[5])
                if cls == 1:
                    label = 'unicorn'
                elif cls == 0:
                    label = 'cat'
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

