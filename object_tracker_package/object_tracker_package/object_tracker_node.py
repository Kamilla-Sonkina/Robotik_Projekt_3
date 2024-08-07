# Datei: /home/joe/ros2_ws/src/Robotik_Projekt_3/object_tracker_package/object_tracker_package/object_tracker_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from object_interfaces.msg import ObjectData
import cv2
import torch
import tensorflow as tf
import time
import numpy as np
from object_tracker_package.ObjectDetector import ObjectDetector
from object_tracker_package.TopDownTransformer import TopDownTransformer
from object_tracker_package.EuclideanDistTracker import EuclideanDistTracker
from object_tracker_package.HelperFunctions import pixel_to_cm, speed_faktor
from object_tracker_package.CoordinateSystem import CoordinateSystem
from object_tracker_package.GlobalCoordinateSystem import GlobalCoordinateSystem
from object_tracker_package.CenterOfMassCalculator import CenterOfMassCalculator

# Transformation toggle
ENABLE_TRANSFORMATION = True

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')

        try:
            # Konfiguration der GPU für TensorFlow
            physical_devices = tf.config.list_physical_devices('GPU')
            if physical_devices:
                for device in physical_devices:
                    tf.config.experimental.set_memory_growth(device, True)
                tf.config.experimental.set_visible_devices(physical_devices[0], 'GPU')
                print(f"Verwende Gerät: {physical_devices[0]}")
        except RuntimeError as e:
            print(e)

        # Konfiguration des Geräts Mac
        if torch.backends.mps.is_available():
            self.device = torch.device('mps')
            print("Verwende Gerät: MPS")
        else:
            self.device = torch.device('cpu')
            print("Verwende Gerät: CPU")

        model_path = "/home/joe/Desktop/runs/runs/detect/train5/weights/best.pt"
        self.detector = ObjectDetector(model_path, self.device)  # Initialisierung des Objektdetektors
        self.transformer = TopDownTransformer()  # Initialisierung des Top-Down-Transformers
        self.global_coordinate_system = GlobalCoordinateSystem()  # Initialisierung des globalen Koordinatensystems
        self.tracker = EuclideanDistTracker(self.global_coordinate_system, speed_faktor)  # Initialisierung des Trackers
        self.coordinate_system = CoordinateSystem(self.transformer, self.global_coordinate_system)  # Initialisierung des Koordinatensystems
        self.center_of_mass_calculator = CenterOfMassCalculator(self.global_coordinate_system)  # Initialisierung des Schwerpunktberechners
        
        video_path = 1  
        #video_path = "/home/joe/Downloads/video.mp4"
        #video_path = "/home/joe/Downloads/video0.mp4"
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Fehler beim Öffnen der Videodatei: {video_path}")
            return
        self.frame_count = 0
        self.results_cache = []
        self.published_ids = {}

        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer für periodische Aufrufe
        self.get_logger().info('Object Tracking Node started.')

        self.object_publisher = self.create_publisher(ObjectData, 'object_data', 10)  # Publisher für Objektdaten
        self.velocity_publisher = self.create_publisher(Float64, 'velocity', 10)  # Publisher für Geschwindigkeit
        self.current_time = 0
        self.grasp_x = -100.0  # ab dieser koordinate wird geschwindigkeit für jeweiliges objekt gepublished 
        self.grasp_y = 0.0
        self.speed_cm = 0.0
        self.average_speed_cm = 0.0

    def timer_callback(self):
        """
        Diese Methode wird periodisch aufgerufen und führt die Objekterkennung und -verfolgung durch.
        """
        if not self.cap.isOpened():
            self.get_logger().error("VideoCapture is not opened")
            self.cap.release()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from video")
            self.cap.release()
            return

        self.current_time = time.time()

        # Frame-Transformation anwenden, falls aktiviert
        if ENABLE_TRANSFORMATION:
            transformed_frame = self.transformer.get_transformed_frame(frame)
        else:
            transformed_frame = frame

        height, width = transformed_frame.shape[:2]

        if self.frame_count % 1 == 0:
            frame_reduced = cv2.resize(transformed_frame, (width // 2, height // 2))
            self.results_cache = self.detector.detect_objects(frame_reduced)  # Objekterkennung durchführen
            self.get_logger().info(f"Detection results: {self.results_cache}")

            objects_rect = []
            for result in self.results_cache:
                if len(result) >= 6:
                    x1, y1, x2, y2 = [int(coord * 2) for coord in result[:4]]
                    w = x2 - x1
                    h = y2 - y1
                    label = 'unicorn' if int(result[5]) == 1 else 'cat'
                    objects_rect.append((x1, y1, w, h, label))
            
            tracked_objects = self.tracker.update(objects_rect, transformed_frame, self.current_time)  # Objekte verfolgen
            self.get_logger().info(f"Tracked objects: {tracked_objects}")

        # Zeichnen der erkannten und verfolgten Objekte
        for i, (x, y, w, h, id) in enumerate(tracked_objects):
            if i < len(objects_rect) and len(objects_rect[i]) >= 5:
                label = objects_rect[i][4]
            else:
                label = 'Unbekannt'

            cv2.rectangle(transformed_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(transformed_frame, f'ID: {id} {label}', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            speed_px = self.tracker.get_speed(id)  # Geschwindigkeit des Objekts ermitteln
            self.speed_cm = pixel_to_cm(speed_px, self.coordinate_system.marker_size)  # Geschwindigkeit in cm/s umrechnen
            grasp_point = self.center_of_mass_calculator.calculate_center_of_mass(transformed_frame, x, y, w, h)  # Greifpunkt berechnen
            
            if self.frame_count % 2 == 0:
                if self.grasp_x > -200: 
                    velocity_msg = Float64()
                    velocity_msg.data = float(self.average_speed_cm)
                    self.velocity_publisher.publish(velocity_msg)  # Geschwindigkeit veröffentlichen
                    cv2.putText(transformed_frame, f'Speed: {self.average_speed_cm:.2f} ', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                    
                    if grasp_point:
                        self.grasp_x, self.grasp_y = grasp_point
                        object_data_msg = ObjectData()
                        object_data_msg.index_value = int(id)
                        object_data_msg.object_class = label
                        object_data_msg.timestamp_value = int(self.current_time)
                        object_data_msg.object_pos_x = float(self.grasp_x)
                        object_data_msg.object_pos_y = float(self.grasp_y)
                        self.object_publisher.publish(object_data_msg)  # Objektdaten veröffentlichen
                        self.get_logger().info(f"Published object data: {object_data_msg}")
                        self.published_ids[id] = (self.grasp_x, self.grasp_y)
                        grasp_point = self.published_ids.get(id, None)

            if grasp_point:
                self.grasp_x, self.grasp_y = grasp_point
            else:
                grasp_point = self.center_of_mass_calculator.calculate_center_of_mass(transformed_frame, x, y, w, h)
                if grasp_point:
                    self.grasp_x, self.grasp_y = grasp_point

            if grasp_point:
                # Konvertieren des Greifpunkts in lokale Koordinaten zum Zeichnen
                local_grasp_x, local_grasp_y = self.global_coordinate_system.to_local(self.grasp_x, self.grasp_y)
                
                cv2.circle(transformed_frame, (int(local_grasp_x), int(local_grasp_y)), 5, (0, 0, 255), -1)
                cv2.putText(transformed_frame, f'Grasp Point: ({self.grasp_x}, {self.grasp_y})', (int(local_grasp_x), int(local_grasp_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            cv2.putText(transformed_frame, f'Speed: {self.speed_cm:.2f} cm/s', (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        average_speed_px = self.tracker.get_average_speed()  # Durchschnittsgeschwindigkeit ermitteln
        self.average_speed_cm = pixel_to_cm(average_speed_px, self.coordinate_system.marker_size)
        cv2.putText(transformed_frame, f'Average Speed: {self.average_speed_cm:.2f} cm/s', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        transformed_frame = self.coordinate_system.draw_coordinate_system(transformed_frame)  # Koordinatensystem zeichnen

        cv2.imshow('Frame', transformed_frame)  # Frame anzeigen
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
