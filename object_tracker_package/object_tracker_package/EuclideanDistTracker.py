import numpy as np
import math
from collections import deque
import cv2
from .CenterOfMassCalculator import CenterOfMassCalculator
from .GlobalCoordinateSystem import GlobalCoordinateSystem


class EuclideanDistTracker:
    def __init__(self, global_coordinate_system, speed_factor):
        """
        Initialisiert den EuclideanDistTracker mit einem globalen Koordinatensystem und einem Geschwindigkeitsfaktor.

        Args:
            global_coordinate_system (GlobalCoordinateSystem): Instanz des globalen Koordinatensystems.
            speed_factor (float): Geschwindigkeitsfaktor für die Berechnung der Geschwindigkeit.
        """
        self.global_coordinate_system = global_coordinate_system
        self.speed_factor = speed_factor
        self.__center_points = {}  # Speichert Mittelpunkte zu jeweiliger ID (Dictionary)
        self.__id_count = 0
        self.__positions = {}  # Zeitstempel und Positionen pro ID
        self.__speed_window_size = 2  # Anzahl der Frames für das Glätten der Geschwindigkeit
        self.__speeds = {}  # Speichert Geschwindigkeiten zu jeweiliger ID
        self.__all_speeds = []  # Speichert alle jemals berechneten Geschwindigkeiten
        self.center_of_mass_calculator = CenterOfMassCalculator(global_coordinate_system)  # Initialisierung des CenterOfMassCalculator
        self.__center_data = {}  # Speichert die Daten der Schwerpunkte

    def update(self, objects_rect, frame, current_time):
        """
        Aktualisiert die Tracker-Informationen basierend auf den erkannten Objekten.

        Args:
            objects_rect (list): Liste der erkannten Objekte im Format (x, y, w, h, label).
            frame (numpy.ndarray): Das aktuelle Frame des Videos.
            current_time (float): Der aktuelle Zeitstempel.

        Returns:
            list: Liste der Bounding-Boxen mit zugehörigen IDs.
        """
        objects_bbs_ids = []  # Liste mit den BBoxen und ID

        for rect in objects_rect:
            x, y, w, h, label = rect
            
            # Berechne den Schwerpunkt
            center_of_mass = self.center_of_mass_calculator.calculate_center_of_mass(frame, x, y, w, h)
            if center_of_mass:
                cx, cy = center_of_mass
                self.__center_data[self.__id_count] = (cx, cy)
            else:
                cx, cy = x + w // 2, y + h // 2  # Fallback zum Mittelpunkt des Rechtecks

            cx, cy = self.global_coordinate_system.to_global(cx, cy)
            same_object_detected = False
            
            for id, pt in self.__center_points.items():
                """
                Überprüft, ob das erkannte Objekt zu einem bereits verfolgten Objekt gehört.

                Für jedes erkannte Objekt wird die euklidische Distanz zwischen dem aktuellen Mittelpunkt und den
                gespeicherten Mittelpunkten der bereits verfolgten Objekte berechnet. Wenn die Distanz kleiner als 100 Pixel
                und größer als -10 Pixel ist, wird das Objekt als dasselbe erkannt und die bestehende ID wird beibehalten.
                Die Position des Objekts wird dann aktualisiert.

                Wenn die Distanz zu einem bestehenden Objekt innerhalb dieses Schwellenwerts liegt, wird die ID des 
                bestehenden Objekts zugewiesen und die Position des Objekts aktualisiert. Falls keine Übereinstimmung
                gefunden wird, wird geprüft, ob die x-Koordinate des neuen Objekts innerhalb von 50 Pixeln einer bestehenden 
                x-Koordinate liegt, um eine mögliche Zuordnung zu erleichtern. Falls auch hier keine Übereinstimmung gefunden
                wird, wird dem neuen Objekt eine neue ID zugewiesen.
                """
                dist = math.hypot(cx - pt[0], cy - pt[1])  # Hypotenuse des jetzigen zum letzten Mittelpunkt

                if (dist < 100) and (dist > -10):
                    self.__center_points[id] = (cx, cy)
                    self.__center_data[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True

                    if id in self.__positions:
                        if len(self.__positions[id]) >= self.__speed_window_size:
                            self.__positions[id].popleft()  # Entfernen des ältesten Eintrags
                        self.__positions[id].append((cx, cy, current_time))
                    else:
                        self.__positions[id] = deque(maxlen=self.__speed_window_size)
                        self.__positions[id].append((cx, cy, current_time))

            if not same_object_detected:
                # Prüfen, ob die x-Koordinate des neuen Objekts innerhalb von 50 Pixeln einer bestehenden x-Koordinate liegt
                for existing_id, existing_pt in self.__center_points.items():
                    if abs(cx - existing_pt[0]) <= 50:
                        self.__center_points[existing_id] = (cx, cy)
                        self.__center_data[existing_id] = (cx, cy)
                        objects_bbs_ids.append([x, y, w, h, existing_id])
                        same_object_detected = True

                        if existing_id in self.__positions:
                            if len(self.__positions[existing_id]) >= self.__speed_window_size:
                                self.__positions[existing_id].popleft()
                            self.__positions[existing_id].append((cx, cy, current_time))
                        else:
                            self.__positions[existing_id] = deque(maxlen=self.__speed_window_size)
                            self.__positions[existing_id].append((cx, cy, current_time))
                        break

                if not same_object_detected:
                    self.__center_points[self.__id_count] = (cx, cy)
                    self.__center_data[self.__id_count] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, self.__id_count])
                    self.__positions[self.__id_count] = deque([(cx, cy, current_time)], maxlen=self.__speed_window_size)
                    self.__id_count += 1

        # Rückgabe der Boundingbox jeder ID 
        self.__center_points = {id: self.__center_points[id] for _, _, _, _, id in objects_bbs_ids}
        return objects_bbs_ids

    def get_speed(self, object_id):
        """
        Berechnet die Geschwindigkeit eines Objekts basierend auf seiner ID.

        Args:
            object_id (int): Die ID des Objekts.

        Returns:
            float: Die berechnete Geschwindigkeit des Objekts.
        """
        # Falls ID im Dictionary enthalten 
        if len(self.__positions[object_id]) > 1:
            positions = list(self.__positions[object_id])
            total_dist = sum(math.hypot(positions[i][0] - positions[i-1][0], positions[i][1] - positions[i-1][1]) for i in range(1, len(positions)))
            total_time = sum(positions[i][2] - positions[i-1][2] for i in range(1, len(positions)))
            if total_time > 0:
                speed = (total_dist / total_time)  
                self.__speeds[object_id] = speed
                if (speed > 0):  # Geschwindigkeit glätten durch herausnehmen der Extrema 
                    self.__all_speeds.append(speed)  # Speichert die Geschwindigkeit
                return speed
        return 0

    def get_average_speed(self):
        """
        Berechnet die durchschnittliche Geschwindigkeit aller Objekte.

        Returns:
            float: Die durchschnittliche Geschwindigkeit.
        """
        if len(self.__all_speeds) > 10:
            average_speed = sum(self.__all_speeds) / len(self.__all_speeds)
            return average_speed
        return 0

    def get_object_center(self, object_id):
        """
        Gibt das Zentrum eines Objekts basierend auf seiner ID zurück.

        Args:
            object_id (int): Die ID des Objekts.

        Returns:
            tuple: Die (x, y) Koordinaten des Zentrums des Objekts.
        """
        return self.__center_points.get(object_id)

    def get_center_data(self, object_id):
        """
        Gibt die Schwerpunktdaten eines Objekts basierend auf seiner ID zurück.

        Args:
            object_id (int): Die ID des Objekts.

        Returns:
            tuple: Die (x, y) Koordinaten des Schwerpunkts des Objekts.
        """
        return self.__center_data.get(object_id)
