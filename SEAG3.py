import cv2
import numpy as np
import time
from ultralytics import YOLO
import tensorflow as tf
from screeninfo import get_monitors
from filterpy.kalman import KalmanFilter

# Konfigurationen
FRAMES_ERKANNT = 1
MIN_AREA = 70 * 70
BASE_POSITION_THRESHOLD = 50
MARKER_SIZE = 100
FRAMES_FOR_STABILITY = 10
frames_antil_transform = 10
distance_bisGreifen = 100  # Beispieldistanz
delay_frame = 0.143 # 

DISTANCE_TO_GRIP = 100  # Distance for grip point calculation (in pixels)
VELOCITY_START_POINT = 0  # Start point for velocity calculation (in cm)
VELOCITY_END_POINT = 10    # End point for velocity calculation (in cm)

def get_screen_resolution():
    monitor = get_monitors()[0]
    return monitor.width, monitor.height

# Helper Functions
def pixel_to_cm(pixel):
    return pixel * (6.4/MARKER_SIZE) # Marker eig. 5,2

def cm_to_pixel(cm):
    return cm / (6.4*MARKER_SIZE)

def get_screen_resolution():
    monitor = get_monitors()[0]
    return monitor.width, monitor.height

def calculate_median_matrix(matrices):
    matrices = np.array(matrices)
    median_matrix = np.median(matrices, axis=0)
    return median_matrix

class KalmanVelocityTracker:
    """
    This class uses a Kalman filter to estimate the velocity of an object based on its position measurements.
    """
    def __init__(self):
        self.kf = KalmanFilter(dim_x=2, dim_z=1)  # 2D state (position, velocity), 1D measurement (position)
        self.kf.x = np.array([0., 0.])  # Initial state: position = 0, velocity = 0
        self.kf.F = np.array([[1., 1.], [0., 1.]])  # State transition matrix (constant velocity model)
        self.kf.H = np.array([[1., 0.]])  # Measurement function (we only measure position)
        self.kf.P *= 1000.  # Initial covariance matrix (high uncertainty)
        self.kf.R = 100.  # Measurement noise (adjust based on your sensor accuracy)

    def update(self, position):
        """
        Updates the Kalman filter with a new position measurement and returns the estimated velocity.
        """
        self.kf.predict()  # Predict the next state
        self.kf.update(position)  # Update the state based on the measurement
        return self.kf.x[1]  # Return the estimated velocity

class TrackedObject:
    """
    Represents a tracked object with its position history, timestamps, class label, and ID.
    """
    def __init__(self, position, cls_label, object_id):
        self.positions = [position]
        self.times = [time.time()]
        self.cls_label = cls_label  # Unicorn ('E') or Cat ('K')
        self.object_id = object_id

    def update_position(self, position):
        """Adds a new position and timestamp to the object's history."""
        self.positions.append(position)
        self.times.append(time.time())
        if len(self.positions) > 10:  # Keep a maximum of 10 positions for speed calculation
            self.positions.pop(0)
            self.times.pop(0)

    def current_position(self):
        """Returns the most recent position of the object."""
        return self.positions[-1] if self.positions else None

    def calculate_grip_position(self, distance):
        if len(self.positions) < 2:
            return self.object_id, self.current_position(), self.cls_label
        current_position = np.array(self.positions[-1])
        grip_position = current_position + np.array([distance, 0])  # Offset to the right
        return self.object_id, tuple(grip_position.astype(int)), self.cls_label  # Convert grip position to integers
    
    def calculate_velocity(self):
        """Calculates the average velocity between the last two detected positions."""
        if len(self.positions) < 2:
            return 0

        start_pos = self.positions[-2]
        end_pos = self.positions[-1]

        distance_pixels = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
        distance_cm = pixel_to_cm(distance_pixels)

        time_diff = self.times[-1] - self.times[-2]

        if time_diff == 0:  # Avoid division by zero
            return 0

        velocity = distance_cm / time_diff
        return velocity
    
def draw_boxes_and_track(frame, tracked_objects, results_cache, frame_index, velocity_trackers):
    """
    This function performs object detection, tracking, and velocity calculation on a given frame.
    It draws bounding boxes around detected objects, updates their positions in the tracked_objects dictionary,
    and calculates their velocities using Kalman filters.
    """
    
    # Object Detection (only on specific frames to optimize performance)
    results = results_cache if frame_index % FRAMES_ERKANNT != 0 else model(frame)
    if frame_index % FRAMES_ERKANNT == 0:
        results_cache = results  # Cache the results for subsequent frames

    for result in results:
        for box in result.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = box
            x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)

            # Object Tracking
            object_tracked = False
            object_id = None
            for obj_id, obj_data in tracked_objects.items():
                # Check if the detected object is already being tracked
                if np.linalg.norm(np.array((x, y)) - np.array(obj_data.positions[-1])) < BASE_POSITION_THRESHOLD and obj_data.cls_label == ('E' if cls == 1 else 'K'):
                    object_tracked = True
                    object_id = obj_id
                    break

            # If not tracked, create a new TrackedObject
            if not object_tracked:
                object_id = len(tracked_objects) + 1
                label = 'E' if cls == 1 else 'K'  # Assign label based on class
                tracked_objects[object_id] = TrackedObject(position=(x, y), cls_label=label, object_id=object_id)

            # Update the object's position
            tracked_objects[object_id].update_position((x, y))

            if object_id not in velocity_trackers:
                velocity_trackers[object_id] = KalmanVelocityTracker()

            current_position = tracked_objects[object_id].current_position()
            if current_position:
                current_position_cm = pixel_to_cm(current_position[0])

                velocity = velocity_trackers[object_id].update(current_position_cm)
                cv2.putText(frame, f'Speed: {velocity:.2f} cm/s', (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Draw Bounding Box and Label
            label = tracked_objects[object_id].cls_label
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {object_id}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # Calculate and Print Grip Position (for debugging/testing)
            obj_id, grip_pos, obj_label = tracked_objects[object_id].calculate_grip_position(DISTANCE_TO_GRIP)
            print(f'Object {obj_id} ({obj_label}): Current Position: {tracked_objects[object_id].current_position()} | Grip Position: {grip_pos}')

    return results_cache


class ArucoTransformer:
    def __init__(self, aruco_size=MARKER_SIZE):
        self.aruco_size = aruco_size
        self.matrices = []
        self.stable_matrix = None
        self.num_frames_for_stability = frames_antil_transform

    def process_frame(self, frame, screen_height):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()

        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            marker_corners = corners[0][0]
            marker_size = self.aruco_size
            top_left_x = 2 * marker_size
            top_left_y = screen_height - 2 * marker_size

            top_down_corners = np.array([[top_left_x, top_left_y],
                                         [top_left_x + marker_size, top_left_y],
                                         [top_left_x + marker_size, top_left_y + marker_size],
                                         [top_left_x, top_left_y + marker_size]], dtype=np.float32)

            matrix = cv2.getPerspectiveTransform(marker_corners, top_down_corners)
            self.matrices.append(matrix)

            if len(self.matrices) == self.num_frames_for_stability:
                self.stable_matrix = calculate_median_matrix(self.matrices)
                print("Stable matrix calculated")
                print(self.stable_matrix)

        return self.stable_matrix, ids is not None
    
def calculate_median_matrix(matrices):
    matrices = np.array(matrices)
    median_matrix = np.median(matrices, axis=0)
    return median_matrix

def transform_to_top_down(frame, screen_width, screen_height, stable_matrix):
        """
        Transforms the given frame to a top-down perspective using the provided stable transformation matrix.
        If no stable matrix is available, the original frame is returned.
        """
        if stable_matrix is None:
            return frame
        top_down_image = cv2.warpPerspective(frame, stable_matrix, (screen_width, screen_height))
        return top_down_image

# YOLO Model Loading
model = YOLO("/Users/johanneszotter/Desktop/SemesterIV/Projekt/training/runs/detect/train4/weights/best.pt")
video_path = r"/Users/johanneszotter/Downloads/FärderbandMitArucoGekürzt.mp4"
#video_path = 0
print("YOLO model loaded.")

# Video Capture and Setup
cap = cv2.VideoCapture(video_path)  
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Get video dimensions for window resizing
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Create and resize the display window
cv2.namedWindow('Transformed Frame with Object Detection', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Transformed Frame with Object Detection', width, height)

# Get screen resolution for top-down transformation
screen_width, screen_height = get_screen_resolution()
aruco_transformer = ArucoTransformer()

# Initialize tracking and processing variables
tracked_objects = {}
frame_index = 0
results_cache = []
velocity_trackers = {}
frame_count = 0

# Main Processing Loop
while cap.isOpened():
    start_time = time.time()  # Record the start time for frame processing
    ret, frame = cap.read()
    if not ret:
        break  # Break the loop if there are no more frames to read
    
    # Eingelesene frames reduzueren
    # Skip frames
    if frame_index % 2 != 0:
        frame_index += 1
        continue

    # Aruco Marker Transformation (if markers are found)
    stable_matrix, marker_found = aruco_transformer.process_frame(frame, screen_height)
    if marker_found:
        top_down_frame = transform_to_top_down(frame, screen_width, screen_height, stable_matrix)
    else:
        top_down_frame = frame  # Use the original frame if no marker is found

    # Object Detection, Tracking, and Velocity Calculation
    results_cache = draw_boxes_and_track(top_down_frame, tracked_objects, results_cache, frame_index, velocity_trackers)
    frame_index += 1

    # Display the Transformed Frame
    cv2.imshow('Transformed Frame with Object Detection', top_down_frame)

    # Frame Delay for Real-Time Processing (after initial frames)
    end_time = time.time()
    total_time = end_time - start_time
    if total_time < delay_frame and frame_count > frames_antil_transform:  
        time.sleep(delay_frame - total_time)  # Adjust sleep time to match desired frame rate
    frame_count += 1

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()  # Release the video capture object
cv2.destroyAllWindows()  # Close all OpenCV windows
print("Video processing completed.")  # Indicate successful completion

