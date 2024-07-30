# Datei: /home/joe/ros2_ws/src/Robotik_Projekt_3/object_tracker_package/object_tracker_package/CoordinateSystem.py

import cv2
import numpy as np
from .GlobalCoordinateSystem import GlobalCoordinateSystem

class CoordinateSystem:
    def __init__(self, top_down_transformer, global_coordinate_system, marker_size=200):
        """
        Initialisiert das Koordinatensystem mit dem Top-Down-Transformer und dem globalen Koordinatensystem.

        Args:
            top_down_transformer (TopDownTransformer): Instanz des Top-Down-Transformers.
            global_coordinate_system (GlobalCoordinateSystem): Instanz des globalen Koordinatensystems.
            marker_size (int): Die Größe des Markers in Pixeln.
        """
        self.top_down_transformer = top_down_transformer
        self.global_coordinate_system = global_coordinate_system
        self.marker_size = marker_size
        self.coordinate_system_frame = None

    def create_coordinate_system(self, width, height):
        """
        Erstellt das lokale Koordinatensystem und zeichnet es auf ein Bild der angegebenen Größe.

        Args:
            width (int): Die Breite des Bildes.
            height (int): Die Höhe des Bildes.
        """
        self.coordinate_system_frame = np.zeros((height, width, 3), dtype=np.uint8)
        origin_x, origin_y = self.global_coordinate_system.to_local(0, 0)

        """
        Zeichnet die Achsen des Koordinatensystems.

        Die x-Achse und die y-Achse werden basierend auf dem Ursprung des globalen Koordinatensystems gezeichnet.
        Die Linien werden in rot (RGB: (0, 0, 255)) mit einer Dicke von 5 Pixeln dargestellt.
        """
        cv2.line(self.coordinate_system_frame, (0, origin_y), (width, origin_y), (0, 0, 255), 5)
        cv2.line(self.coordinate_system_frame, (origin_x, 0), (origin_x, height), (0, 0, 255), 5)

    def draw_coordinate_system(self, frame):
        """
        Zeichnet das Koordinatensystem auf das gegebene Frame.

        Falls das Koordinatensystem noch nicht erstellt wurde oder die Größe des Frames sich geändert hat,
        wird es neu erstellt. Das Koordinatensystem wird dann mit dem aktuellen Frame kombiniert.

        Args:
            frame (numpy.ndarray): Das Eingabeframe.

        Returns:
            numpy.ndarray: Das kombinierte Frame mit dem gezeichneten Koordinatensystem.
        """
        height, width = frame.shape[:2]
        if self.coordinate_system_frame is None or self.coordinate_system_frame.shape[:2] != (height, width):
            self.create_coordinate_system(width, height)
        combined_frame = cv2.addWeighted(frame, 1, self.coordinate_system_frame, 1, 0)
        return combined_frame
