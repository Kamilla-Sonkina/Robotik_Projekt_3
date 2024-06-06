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

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Verwende Gerät: {device}")

        model_path = "/home/markus/Desktop/runs/detect/train5/weights/best.pt"
        self.detector = ObjectDetector(model_path, device)
        self.transformer = TopDownTransformer()
        self.tracker = EuclideanDistTracker()

        video_path = "/home/markus/Downloads/test_video.mov"
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

        current_time = time.time()

        frame = self.transformer.get_transformed_frame(frame)
        height, width = frame.shape[:2]

        frame_reduced = cv2.resize(frame, (width // 2, height // 2))
        self.results_cache = self.detector.detect_objects(frame_reduced)

        objects_rect = []
        for result in self.results_cache:
            if len(result) >= 6:
                x1, y1, x2, y2 = [int(coord * 2) for coord in result[:4]]
                w = x2 - x1
                h = y2 - y1
                label = 'unicorn' if int(result[5]) == 1 else 'cat'
                objects_rect.append((x1, y1, w, h, label))

        tracked_objects = self.tracker.update([rect[:4] for rect in objects_rect], current_time)

        for i, (x, y, w, h, id) in enumerate(tracked_objects):
            label = objects_rect[i][4]
            cx = x + w // 2
            cy = y + h // 2

            object_msg = ObjectData()
            object_msg.object_pos_x = float(cx)
            object_msg.object_pos_y = float(cy)
            object_msg.object_class = label
            object_msg.timestamp_value = float(current_time)
            object_msg.index_value = int(id)
            self.object_publisher.publish(object_msg)
            self.get_logger().info(f"object is at x: {object_msg.object_pos_x}, y: {object_msg.object_pos_y}, the class is: {object_msg.object_class}, the id is: {object_msg.index_value}")

            speed = self.tracker.get_speed(id)
            speed_msg = Float64()
            speed_msg.data = float(speed)
            self.velocity_publisher.publish(speed_msg)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f'ID: {id} {label}', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.putText(frame, f'Speed: {speed:.2f} cm/s', (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            center_position = self.tracker.get_object_center(id)
                if center_position:
                    cv2.circle(frame, center_position, 5, (0, 0, 255), -1)

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


