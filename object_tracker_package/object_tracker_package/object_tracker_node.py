import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import time
from object_interfaces.msg import ObjectData

class ObjectTrackerNode(Node):
    class TrackedObject:
        def __init__(self, position, bbox):
            self.positions = [position]
            self.bbox = bbox
            self.speeds = []
            self.last_update_frame = 0

        def update_position(self, new_position, frame_index):
            self.positions.append(new_position)
            self.last_update_frame = frame_index
            if len(self.positions) > 1:
                speed = np.linalg.norm(np.array(new_position) - np.array(self.positions[-2]))
                self.speeds.append(speed)

        def average_speed(self):
            if self.speeds:
                return sum(self.speeds) / len(self.speeds)
            return 0

    def __init__(self):
        super().__init__('object_tracker_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(ObjectData, 'object_data', 10)
        
        # Load YOLO model
        self.model = YOLO("/Users/johanneszotter/Desktop/SemesterIV/Projekt/training/runs/detect/train4/weights/best.pt")

        self.frames_detected = 1
        self.min_area = 70 * 70
        self.base_position_threshold = 20
        self.src_points = np.float32([[420, 655], [1140, 680], [410, 843], [1125, 835]])
        self.dst_points = np.float32([[420, 655], [1125, 655], [420, 843], [1125, 843]])
        self.box_boundary_right = 1000
        self.tracked_objects = {}
        self.frame_index = 0
        self.results_cache = []

        # Initialize CV Bridge
        self.bridge = CvBridge()

    def transform_perspective(self, frame):
        matrix = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        result_frame = cv2.warpPerspective(frame, matrix, (frame.shape[1], frame.shape[0]))
        return result_frame

    def draw_boxes_and_track(self, frame):
        # Draw detection rectangle
        top_left = (int(self.dst_points[0][0]), int(self.dst_points[0][1]))
        bottom_right = (int(self.dst_points[3][0] + self.box_boundary_right), int(self.dst_points[3][1]))
        cv2.rectangle(frame, top_left, bottom_right, (255, 0, 0), 3)  # Red outline, 3 pixels thick

        # YOLO-based unicorn and cat detection
        results = self.results_cache if self.frame_index % self.frames_detected != 0 else self.model(frame)
        if self.frame_index % self.frames_detected == 0:
            self.results_cache = results  # Cache results

        for result in results:
            for box in result.boxes.data.tolist():
                x1, y1, x2, y2, conf, cls = box
                x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)

                # Check if the detected object is within the defined box
                if x > top_left[0] and y > top_left[1] and x + w < bottom_right[0] and y + h < bottom_right[1]:
                    label = 'Unicorn' if cls == 1 else 'Cat'
                    # Tracking logic
                    object_tracked = False
                    object_id = None
                    for obj_id, obj_data in self.tracked_objects.items():
                        if np.linalg.norm(np.array((x, y)) - np.array(obj_data.position)) < self.base_position_threshold:
                            object_tracked = True
                            object_id = obj_id
                            break
                    if not object_tracked:
                        object_id = len(self.tracked_objects) + 1
                        self.tracked_objects[object_id] = self.TrackedObject(position=(x, y), bbox=(x, y, w, h))
                    self.tracked_objects[object_id].update_position((x, y), self.frame_index)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, f'{label}: Avg. Speed: {self.tracked_objects[object_id].average_speed()}',
                                (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                    
                    # Publish object data
                    object_data = ObjectData()
                    object_data.object_pos_x = float(x)
                    object_data.object_pos_y = float(y)
                    object_data.object_class = label
                    object_data.timestamp_value = self.get_clock().now().to_msg().sec
                    object_data.index_value = object_id
                    self.publisher.publish(object_data)
                    
        self.frame_index += 1
        return frame

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        transformed_frame = self.transform_perspective(frame)
        tracked_frame = self.draw_boxes_and_track(transformed_frame)
        cv2.imshow('Transformed Frame with Object Detection', tracked_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    object_tracker_node = ObjectTrackerNode()
    rclpy.spin(object_tracker_node)
    object_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
