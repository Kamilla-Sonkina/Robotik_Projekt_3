# Datei: /home/joe/ros2_ws/src/Robotik_Projekt_3/object_tracker_package/object_tracker_package/GlobalCoordinateSystem.py

import numpy as np

class GlobalCoordinateSystem:
    origin_x = 757  # Koordinatensystem kalibrieren auf roten Punkt auf dem Lineal neben Fließband
    origin_y = 362  # Koordinatensystem kalibrieren auf roten Punkt auf dem Lineal neben Fließband

    def __init__(self):
        self.origin_x = GlobalCoordinateSystem.origin_x
        self.origin_y = GlobalCoordinateSystem.origin_y

    def to_global(self, local_x, local_y):
        """
        Konvertiert lokale Koordinaten in globale Koordinaten.

        Args:
            local_x (int): Lokale x-Koordinate.
            local_y (int): Lokale y-Koordinate.

        Returns:
            tuple: Globale (x, y) Koordinaten.
        """
        global_x = local_x - self.origin_x
        global_y = local_y - self.origin_y
        return (global_x, global_y)

    def to_local(self, global_x, global_y):
        """
        Konvertiert globale Koordinaten in lokale Koordinaten.

        Args:
            global_x (int): Globale x-Koordinate.
            global_y (int): Globale y-Koordinate.

        Returns:
            tuple: Lokale (x, y) Koordinaten.
        """
        local_x = global_x + self.origin_x
        local_y = global_y + self.origin_y
        return (local_x, local_y)

