import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from object_interfaces.msg import ObjectData
import cv2
import torch
import tensorflow as tf
import time
import numpy as np
from .ObjectDetector import ObjectDetector
from .TopDownTransformer import TopDownTransformer
from .EuclideanDistTracker import EuclideanDistTracker
from .HelperFunctions import pixel_to_cm
from .CoordinateSystem import CoordinateSystem
from .CenterOfMassCalculator import CenterOfMassCalculator

# Transformation toggle
ENABLE_TRANSFORMATION = True

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')
        
        try:
            physical_devices = tf.config.list_physical_devices('GPU')
            if physical_devices:
                for device in physical_devices:
                    tf.config.experimental.set_memory_growth(device, True)
                tf.config.experimental.set_visible_devices(physical_devices[0], 'GPU')
                print(f"Verwende Gerät: {physical_devices[0]}")
        except RuntimeError as e:
            print(e)

        if torch.backends.mps.is_available():
            self.device = torch.device('mps')
            print("Verwende Gerät: MPS")
        else:
            self.device = torch.device('cpu')
            print("Verwende Gerät: CPU")

        model_path = "/home/joe/Desktop/runs/runs/detect/train5/weights/best.pt"
        self.detector = ObjectDetector(model_path, self.device)
        self.transformer = TopDownTransformer()
        self.tracker = EuclideanDistTracker()
        self.coordinate_system = CoordinateSystem(self.transformer)
        self.center_of_mass_calculator = CenterOfMassCalculator()

        #video_path = 1  # Use this if using a webcam
        video_path = "/home/joe/Downloads/video.mp4"
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Fehler beim Öffnen der Videodatei: {video_path}")
            return
        self.frame_count = 0
        self.results_cache = []
        self.published_ids = {}

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Object Tracking Node started.')

        self.object_publisher = self.create_publisher(ObjectData, 'object_data', 10)
        self.velocity_publisher = self.create_publisher(Float64, 'velocity', 10)
        self.current_time = 0
        self.grasp_x = 0.0
        self.grasp_y = 0.0
        self.speed_cm = 0.0

    def timer_callback(self):
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

        # Apply transformation if enabled
        if ENABLE_TRANSFORMATION:
            transformed_frame = self.transformer.get_transformed_frame(frame)
        else:
            transformed_frame = frame

        height, width = transformed_frame.shape[:2]

        if self.frame_count % 1 == 0:
            frame_reduced = cv2.resize(transformed_frame, (width // 2, height // 2))
            self.results_cache = self.detector.detect_objects(frame_reduced)
            self.get_logger().info(f"Detection results: {self.results_cache}")

            objects_rect = []
            for result in self.results_cache:
                if len(result) >= 6:
                    x1, y1, x2, y2 = [int(coord * 2) for coord in result[:4]]
                    w = x2 - x1
                    h = y2 - y1
                    label = 'unicorn' if int(result[5]) == 1 else 'cat'
                    objects_rect.append((x1, y1, w, h, label))
            
            tracked_objects = self.tracker.update(objects_rect, transformed_frame, self.current_time)
            self.get_logger().info(f"Tracked objects: {tracked_objects}")

        for i, (x, y, w, h, id) in enumerate(tracked_objects):
            if i < len(objects_rect) and len(objects_rect[i]) >= 5:
                label = objects_rect[i][4]
            else:
                label = 'Unbekannt'

            cv2.rectangle(transformed_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(transformed_frame, f'ID: {id} {label}', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            speed_px = self.tracker.get_speed(id)
            self.speed_cm = pixel_to_cm(speed_px, self.coordinate_system.marker_size)
            grasp_point = self.center_of_mass_calculator.calculate_center_of_mass(transformed_frame, x, y, w, h)
            
            if self.frame_count % 2 == 0:
                # Publish velocity
                if self.grasp_x > 200:  # Check if the x value has crossed 300
                    velocity_msg = Float64()
                    velocity_msg.data = float(self.speed_cm)
                    self.velocity_publisher.publish(velocity_msg)
                    cv2.putText(transformed_frame, f'Speed: {self.speed_cm:.2f} ',(20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                    
                    if grasp_point:
                        self.grasp_x, self.grasp_y = grasp_point
                        object_data_msg = ObjectData()
                        object_data_msg.index_value = int(id)
                        object_data_msg.object_class = label
                        object_data_msg.timestamp_value = int(self.current_time)
                        object_data_msg.object_pos_x = float(self.grasp_x)
                        object_data_msg.object_pos_y = float(self.grasp_y)
                        self.object_publisher.publish(object_data_msg)
                        self.get_logger().info(f"Published object data: {object_data_msg}")
                        self.published_ids[id] = (self.grasp_x, self.grasp_y)
                        grasp_point = self.published_ids.get(id, None)
                        
            #Display grasp point
            #grasp_point = self.published_ids.get(id, None)
            if grasp_point:
                self.grasp_x, self.grasp_y = grasp_point
            else:
                grasp_point = self.center_of_mass_calculator.calculate_center_of_mass(transformed_frame, x, y, w, h)
                if grasp_point:
                    self.grasp_x, self.grasp_y = grasp_point

            if grasp_point:
                grasp_x_cm = pixel_to_cm(self.grasp_x, self.coordinate_system.marker_size)
                grasp_y_cm = pixel_to_cm(self.grasp_y, self.coordinate_system.marker_size)
                
                cv2.circle(transformed_frame, (int(self.grasp_x), int(self.grasp_y)), 5, (0, 0, 255), -1)
                cv2.putText(transformed_frame, f'Grasp Point: ({self.grasp_x}, {self.grasp_y})', (int(self.grasp_x) , int(self.grasp_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1) #grasp_x_cm:.2f, grasp_y_cm:.2f

            cv2.putText(transformed_frame, f'Speed: {self.speed_cm:.2f} cm/s', (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        average_speed_px = self.tracker.get_average_speed()
        average_speed_cm = pixel_to_cm(average_speed_px, self.coordinate_system.marker_size)
        cv2.putText(transformed_frame, f'Average Speed: {average_speed_cm:.2f} cm/s',(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        transformed_frame = self.coordinate_system.draw_coordinate_system(transformed_frame)

        cv2.imshow('Frame', transformed_frame)
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
