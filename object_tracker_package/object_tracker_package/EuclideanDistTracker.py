import numpy as np
import math
from collections import deque
import cv2
from CenterOfMassCalculator import CenterOfMassCalculator  # Import der Klasse

class EuclideanDistTracker:
    def __init__(self):
        self.__center_points = {}  # Speichert Mittelpunkte zu jeweiliger ID (Dictionary)
        self.__id_count = 0
        self.__positions = {}  # Zeitstempel und Positionen pro ID
        self.__speed_window_size = 2  # Anzahl der Frames für das Glätten der Geschwindigkeit
        self.__speeds = {}  # Speichert Geschwindigkeiten zu jeweiliger ID
        self.center_of_mass_calculator = CenterOfMassCalculator()  # Initialisierung des CenterOfMassCalculator

    def update(self, objects_rect, frame, current_time): 
        objects_bbs_ids = []  # Liste mit den BBoxen und ID

        # Itereriren über Objekte
        for rect in objects_rect:
            x, y, w, h, label = rect  
            
            # Berechne den Masseschwerpunkt
            center_of_mass = self.center_of_mass_calculator.max_inscribed_circle(frame, x, y, w, h)
            if center_of_mass:
                cx, cy = center_of_mass[0], center_of_mass[1]
            else:
                cx, cy = x + w // 2, y + h // 2  # Fallback zum Mittelpunkt des Rechtecks

            same_object_detected = False
            for id, pt in self.__center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])  # Hypotenuse des jetzigen zum letzten Mittelpunkt

                if dist < 100:
                    self.__center_points[id] = (cx, cy)
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
                self.__center_points[self.__id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.__id_count])
                self.__positions[self.__id_count] = deque([(cx, cy, current_time)], maxlen=self.__speed_window_size)
                self.__id_count += 1

        # Rückgabe der Boundingbox jeder ID 
        self.__center_points = {id: self.__center_points[id] for _, _, _, _, id in objects_bbs_ids}
        return objects_bbs_ids

    def get_speed(self, object_id):
        # Falls ID im Dictionary enthalten 
        if len(self.__positions[object_id]) > 1:
            positions = list(self.__positions[object_id])
            total_dist = sum(math.hypot(positions[i][0] - positions[i-1][0], positions[i][1] - positions[i-1][1]) for i in range(1, len(positions)))
            total_time = sum(positions[i][2] - positions[i-1][2] for i in range(1, len(positions)))
            if total_time > 0:
                speed = (total_dist / total_time)  
                self.__speeds[object_id] = speed
                return speed
        return 0

    def get_average_speed(self):
        if len(self.__speeds) > 0:
            average_speed = sum(self.__speeds.values()) / len(self.__speeds)
            return average_speed
        return 0

    def get_object_center(self, object_id):
        return self.__center_points.get(object_id)


    def get_object_center(self, object_id):
        if object_id in self.__center_points:
            return self.__center_points[object_id]
        return None
