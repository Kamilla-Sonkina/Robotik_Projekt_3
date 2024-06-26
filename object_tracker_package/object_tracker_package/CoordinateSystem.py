import cv2
import numpy as np

class CoordinateSystem:
    def __init__(self, top_down_transformer, marker_size=100, origin_x=184, origin_y=454):
        self.top_down_transformer = top_down_transformer
        self.marker_size = marker_size
        self.coordinate_system_frame = None
        self.origin_x = origin_x
        self.origin_y = origin_y

    def transform_point(self, point):
        stable_matrix = self.top_down_transformer.stable_matrix
        if stable_matrix is not None:
            point = np.array([point], dtype=np.float32)
            point = np.array([point])
            transformed_point = cv2.perspectiveTransform(point, stable_matrix)
            return transformed_point[0][0]
        return point

    def create_coordinate_system(self, width, height):
        self.coordinate_system_frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Verwende die angegebenen Ursprungskoordinaten oder standardmäßig die Mitte
        origin_x = self.origin_x if self.origin_x is not None else width // 2
        origin_y = self.origin_y if self.origin_y is not None else height // 2
        
        # Zeichne die X-Achse
        cv2.line(self.coordinate_system_frame, (0, origin_y), (width, origin_y), (255, 0, 0), 2)
        
        # Zeichne die Y-Achse
        cv2.line(self.coordinate_system_frame, (origin_x, 0), (origin_x, height), (255, 0, 0), 2)

    def draw_coordinate_system(self, frame):
        height, width = frame.shape[:2]
        if self.coordinate_system_frame is None or self.coordinate_system_frame.shape[:2] != (height, width):
            self.create_coordinate_system(width, height)
        
        combined_frame = cv2.addWeighted(frame, 1, self.coordinate_system_frame, 1, 0)
        return combined_frame
