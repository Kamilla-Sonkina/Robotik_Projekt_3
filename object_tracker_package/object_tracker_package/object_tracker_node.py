import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from object_interfaces.msg import ObjectData
import cv2
import torch
import time
from object_tracker_package.ObjectDetector import ObjectDetector
from object_tracker_package.TopDownTransformer import TopDownTransformer
from object_tracker_package.EuclideanDistTracker import EuclideanDistTracker
from object_tracker_package.HelperFunctions import pixel_to_cm
from object_tracker_package.CoordinateSystem import CoordinateSystem
from object_tracker_package.CenterOfMassCalculator import CenterOfMassCalculator

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Verwende Gerät: {device}")

        model_path = "/home/markus/Desktop/runs/detect/train5/weights/best.pt"
        self.detector = ObjectDetector(model_path, device)
        self.transformer = TopDownTransformer()
        self.tracker = EuclideanDistTracker()
        self.coordinate_system = CoordinateSystem(self.transformer)
        self.center_of_mass_calculator = CenterOfMassCalculator(self.transformer)

        video_path = "/home/markus/Downloads/test_video.mov"
        #video_path=2
        self.cap = cv2.VideoCapture(video_path)
        self.frame_count = 0
        self.results_cache = []
        

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.object_publisher = self.create_publisher(ObjectData, 'object_data', 10)
        self.velocity_publisher = self.create_publisher(Float64, 'velocity', 10)

    def timer_callback(self):
        if not self.cap.isOpened():
            self.cap.release()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.cap.release()
            return

        #current_time = time.time()
        current_time = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

        # Transformiere das Bild in eine Top-Down-Ansicht
        transformed_frame = self.transformer.get_transformed_frame(frame)
        height, width = transformed_frame.shape[:2]

        if self.frame_count % 1 == 0:
            frame_reduced = cv2.resize(transformed_frame, (width // 2, height // 2))
            results_cache = self.detector.detect_objects(frame_reduced)
            print(f"Ergebnisse auf Frame {self.frame_count}: {results_cache}")

            # Konvertiere die Ergebnisse in das Format (x, y, w, h, label)
            objects_rect = []
            for result in results_cache:
                if len(result) >= 6:
                    x1, y1, x2, y2 = [int(coord * 2) for coord in result[:4]]
                    w = x2 - x1
                    h = y2 - y1
                    label = 'Einhorn' if int(result[5]) == 1 else 'Katze'
                    objects_rect.append((x1, y1, w, h, label))
            
            # Aktualisiere den Tracker
            tracked_objects = self.tracker.update(objects_rect, transformed_frame, current_time)

        # Zeichne die verfolgten Objekte
        for i, (x, y, w, h, id) in enumerate(tracked_objects):
            if i < len(objects_rect) and len(objects_rect[i]) >= 5:
                label = objects_rect[i][4]  # Label aus den Objekterkennungsergebnissen
            else:
                label = 'Unbekannt'

            cv2.rectangle(transformed_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(transformed_frame, f'ID: {id} {label}', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Geschwindigkeit anzeigen
            speed_px = self.tracker.get_speed(id)
            speed_cm = pixel_to_cm(speed_px, self.coordinate_system.marker_size)
            cv2.putText(transformed_frame, f'Speed: {speed_cm:.2f} cm/s', (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            
            # Zeichne den größtmöglichen einbeschriebenen Kreis und den Mittelpunkt
            circle_data = center_of_mass_calculator.max_inscribed_circle(transformed_frame, x, y, w, h)
            if circle_data:
                circle_x, circle_y, radius = circle_data
                circle_x_cm = pixel_to_cm(circle_x, coordinate_system.marker_size)
                circle_y_cm = pixel_to_cm(circle_y, coordinate_system.marker_size)
                radius_cm = pixel_to_cm(radius, coordinate_system.marker_size)
                cv2.circle(transformed_frame, (circle_x, circle_y), radius, (255, 255, 0), 2)
                cv2.putText(transformed_frame, f'Circle Center: ({circle_x_cm:.2f} cm, {circle_y_cm:.2f} cm)', (circle_x + 10, circle_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                # Zeichne den Mittelpunkt
                cv2.circle(transformed_frame, (circle_x, circle_y), 5, (0, 0, 255), -1)

        # Durchschnittsgeschwindigkeit aller Objekte berechnen und anzeigen
        average_speed_px = tracker.get_average_speed()
        average_speed_cm = pixel_to_cm(average_speed_px, coordinate_system.marker_size)
        cv2.putText(transformed_frame, f'Average Speed: {average_speed_cm:.2f} cm/s', (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # Zeichne das Koordinatensystem
        transformed_frame = coordinate_system.draw_coordinate_system(transformed_frame)

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



