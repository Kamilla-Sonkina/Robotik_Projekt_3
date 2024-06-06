import numpy as np
import math
from collections import deque

class EuclideanDistTracker:
    def __init__(self):
        self.__center_points = {} # Speichert mittelpunkte zu jeweiliger ID (Dictionary)
        self.__id_count = 0
        self.__positions = {} # zeitstempel und Position und Schlüssel ID
        self.__speed_window_size = 5  # Number of frames for smoothing

    # verwaltung der ID, Koordinaten und Mittelpunkt 
    def update(self, objects_rect, current_time):
        objects_bbs_ids = [] # lisste mit BBoxen und ID

        for rect in objects_rect:
            x, y, w, h = rect # def rechteck
            cx = (x + x + w) // 2 # Mittelpunkt BBox
            cy = (y + y + h) // 2

            same_object_detected = False

            # a
            for id, pt in self.__center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1]) # hypotenuse des jetztigen zum letzten mittelpunkt

                # abfrage ob selbes objekt: falls altes objekt wird der ID neue position und zeit zugeördnet 
                if dist < 100:
                    self.__center_points[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id]) # boundingbox zur ID hinzufügen
                    same_object_detected = True

                    # Position und Zeitstempel speichern
                    if id in self.__positions: # überprüfen ob ID bereitz vorhanden
                        if len(self.__positions[id]) >= self.__speed_window_size: # Üb. mehr als 5 positionen gespeichert
                            self.__positions[id].popleft() # entfernen des ältersten eintrags 
                        self.__positions[id].append((cx, cy, current_time)) # neue position und Zeit speichern
                    else:
                        self.__positions[id] = deque(maxlen=self.__speed_window_size) # neue deque für id 
                        self.__positions[id].append((cx, cy, current_time)) # neue position und Zeit speichern
            
            # falls neues objekt: neue ID + werte 
            if not same_object_detected: 
                self.__center_points[self.__id_count] = (cx, cy) # neuer Mittelpunkt
                objects_bbs_ids.append([x, y, w, h, self.__id_count]) # Bbox und ID hinzufügen
                self.__positions[self.__id_count] = deque([(cx, cy, current_time)], maxlen=self.__speed_window_size)
                self.__id_count += 1

        # Alte getrackte Objekte werden aktualisiert 
        new_center_points = {} # speichern der Mittelpunkte 
        for obj_bb_id in objects_bbs_ids:  # schleife über verfolgte Objetke
            _, _, _, _, object_id = obj_bb_id 
            center = self.__center_points[object_id] # letzt berechneter mittelpunkt für ID
            new_center_points[object_id] = center

        self.__center_points = new_center_points.copy()
        return objects_bbs_ids

    def get_speed(self, object_id):
        if object_id in self.__positions and len(self.__positions[object_id]) > 1: # Prüfen ob Position vorhanden
            positions = list(self.__positions[object_id]) # lisste einlesen
            total_dist = 0
            total_time = 0

            for i in range(1, len(positions)): 
                prev_cx, prev_cy, prev_time = positions[i - 1] #letzte posisiton und zeit 
                curr_cx, curr_cy, curr_time = positions[i] # aktuelle position und Zeit
                dist = math.hypot(curr_cx - prev_cx, curr_cy - prev_cy) # distanz berechnen
                time_elapsed = curr_time - prev_time # vergangene zeit 
                total_dist += dist 
                total_time += time_elapsed

            if total_time > 0: 
                average_speed = total_dist / total_time 
                return average_speed * 0.052

        return 0

    def get_object_center(self, object_id):
        if object_id in self.__center_points:
            return self.__center_points[object_id]
        return None
