import numpy as np
import math
from collections import deque

class EuclideanDistTracker:
    def __init__(self):
        self.__center_points = {}
        self.__id_count = 0
        self.__id_prev = -1
        self.__positions = {}
        self.__speed_window_size = 5  # Number of frames for smoothing

    def update(self, objects_rect, current_time):
        objects_bbs_ids = []

        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            same_object_detected = False

            for id, pt in self.__center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 100:
                    self.__center_points[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True

                    # Position und Zeitstempel speichern
                    if id in self.__positions:
                        if len(self.__positions[id]) >= self.__speed_window_size:
                            self.__positions[id].popleft()
                        self.__positions[id].append((cx, cy, current_time))
                    else:
                        self.__positions[id] = deque(maxlen=self.__speed_window_size)
                        self.__positions[id].append((cx, cy, current_time))

            if not same_object_detected:
                self.__center_points[self.__id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.__id_count])
                self.__positions[self.__id_count] = deque([(cx, cy, current_time)], maxlen=self.__speed_window_size)
                self.__id_count += 1

        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.__center_points[object_id]
            new_center_points[object_id] = center

        self.__center_points = new_center_points.copy()
        return objects_bbs_ids

    def get_speed(self, object_id):
        if object_id in self.__positions and len(self.__positions[object_id]) > 1:
            positions = list(self.__positions[object_id])
            total_dist = 0
            total_time = 0

            for i in range(1, len(positions)):
                prev_cx, prev_cy, prev_time = positions[i - 1]
                curr_cx, curr_cy, curr_time = positions[i]
                dist = math.hypot(curr_cx - prev_cx, curr_cy - prev_cy)
                time_elapsed = curr_time - prev_time
                total_dist += dist
                total_time += time_elapsed

            if total_time > 0:
                average_speed = total_dist / total_time
                return average_speed*0.025

        return 0