import cv2
import numpy as np
import tkinter as tk

class TopDownTransformer:
        
    def __init__(self, marker_size=200, num_frames_for_stability=30, crop_height_fraction=0.5, upper_crop_ratio=0.5, lower_crop_ratio=0.5):
        self.marker_size = marker_size
        self.num_frames_for_stability = num_frames_for_stability
        self.screen_width, self.screen_height = self.get_screen_resolution()
        self.stable_matrix = None
        self.matrices = []
        self.frame_count = 0
        self.crop_height_fraction = crop_height_fraction  # Fraction of the height to keep in the center
        self.upper_crop_ratio = upper_crop_ratio  # Proportion of the crop from the top
        self.lower_crop_ratio = lower_crop_ratio  # Proportion of the crop from the bottom

    def get_screen_resolution(self):
        root = tk.Tk()
        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()
        root.destroy()
        return width, height

    def calculate_median_matrix(self, matrices):
        matrices = np.array(matrices)
        median_matrix = np.median(matrices, axis=0)
        return median_matrix

    def transform_to_top_down(self, frame):
        if self.stable_matrix is None:
            return frame
        
        top_down_image = cv2.warpPerspective(frame, self.stable_matrix, (self.screen_width, self.screen_height))
        
        # Calculate crop boundaries based on new upper and lower crop ratios
        total_crop_height = self.screen_height * (1 - self.crop_height_fraction)
        upper_crop_height = int(total_crop_height * self.upper_crop_ratio)
        lower_crop_height = int(total_crop_height * self.lower_crop_ratio)

        y_start = upper_crop_height
        y_end = self.screen_height - lower_crop_height
        
        cropped_image = top_down_image[y_start:y_end, :]
        
        return cropped_image

    def update_transformation_matrix(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            marker_corners = corners[0][0]
            top_left_x = 0#self.marker_size
            top_left_y = self.screen_height - 2 * self.marker_size

            top_down_corners = np.array([
                [top_left_x, top_left_y],
                [top_left_x + self.marker_size, top_left_y],
                [top_left_x + self.marker_size, top_left_y + self.marker_size],
                [top_left_x, top_left_y + self.marker_size]
            ], dtype=np.float32)

            matrix = cv2.getPerspectiveTransform(marker_corners, top_down_corners)
            self.matrices.append(matrix)

            if self.frame_count == self.num_frames_for_stability:
                self.stable_matrix = self.calculate_median_matrix(self.matrices)
                print("Stabile Matrix berechnet")

            self.frame_count += 1

    def get_transformed_frame(self, frame):
        self.update_transformation_matrix(frame)
        return self.transform_to_top_down(frame)
