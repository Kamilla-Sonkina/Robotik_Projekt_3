# Datei: /home/joe/ros2_ws/src/Robotik_Projekt_3/object_tracker_package/object_tracker_package/CenterOfMassCalculator.py

import cv2
import numpy as np
from .GlobalCoordinateSystem import GlobalCoordinateSystem

class CenterOfMassCalculator:
    def __init__(self, global_coordinate_system):
        """
        Initialisiert den CenterOfMassCalculator mit einem globalen Koordinatensystem.

        Args:
            global_coordinate_system (GlobalCoordinateSystem): Instanz des globalen Koordinatensystems.
        """
        self.global_coordinate_system = global_coordinate_system

    def calculate_center_of_mass(self, image, x, y, w, h):
        """
        Berechnet den Schwerpunkt eines Objekts innerhalb eines gegebenen Bildausschnitts und passt diesen an,
        um eine stabilere Greifposition zu gewährleisten.

        Args:
            image (numpy.ndarray): Das Eingabebild.
            x (int): Die x-Koordinate der oberen linken Ecke des Bildausschnitts.
            y (int): Die y-Koordinate der oberen linken Ecke des Bildausschnitts.
            w (int): Die Breite des Bildausschnitts.
            h (int): Die Höhe des Bildausschnitts.

        Returns:
            tuple: Die globalen (x, y) Koordinaten des berechneten Schwerpunkts.
        """
        # ROI (Region of Interest) ausschneiden und in Graustufen umwandeln
        object_roi = image[y:y+h, x:x+w]
        gray_roi = cv2.cvtColor(object_roi, cv2.COLOR_BGR2GRAY)
        
        # Bild glätten, um Rauschen zu reduzieren
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        
        # Binärisierung des Bildes mit Otsu's Schwellenwertmethode
        _, binary_roi = cv2.threshold(blurred_roi, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    
        # Anwendung morphologischer Operationen zur Bildsäuberung
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cleaned = cv2.morphologyEx(binary_roi, cv2.MORPH_CLOSE, kernel, iterations=2)
    
        # Konturen finden
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None
        
        # Berechnung der Momente zur Bestimmung des Schwerpunkts
        moments = cv2.moments(cleaned)
        if moments["m00"] != 0:
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
        else:
            center_x, center_y = 0, 0

        # Berechnung des Schwerpunkts im gesamten Bild und Umrechnung in globale Koordinaten
        global_center_x, global_center_y = int(center_x + x), int(center_y + y)
        global_center_x, global_center_y = self.global_coordinate_system.to_global(global_center_x, global_center_y)

        """
        Finden des Punktes mit dem geringsten Abstand zum Rand des Objekts.

        Der Punkt innerhalb der Konturen, der den minimalen Abstand zum Schwerpunkt hat, wird ermittelt.
        Dieser Punkt dient dazu, die Richtung der Verschiebung des Schwerpunkts zu bestimmen,
        um eine stabilere Greifposition zu gewährleisten.
        """
        min_distance = float('inf')
        closest_point = None
        for contour in contours:
            for point in contour:
                px, py = point[0]
                distance = np.sqrt((center_x - px) ** 2 + (center_y - py) ** 2)
                if distance < min_distance:
                    min_distance = distance
                    closest_point = (px, py)

        """
        Anpassung des Schwerpunkts basierend auf dem minimalen Abstand zum Rand.

        Der Schwerpunkt wird um einen kleinen Betrag (10 Pixel) in die entgegengesetzte Richtung des geringsten Abstands
        verschoben, um eine robustere Greifposition zu erhalten.
        """
        if closest_point:
            px, py = closest_point
            dx = center_x - px
            dy = center_y - py
            distance = np.sqrt(dx ** 2 + dy ** 2)
            if distance > 0:
                shift_x = int((dx / distance) * 10)
                shift_y = int((dy / distance) * 10)
                shifted_center_x = center_x + shift_x
                shifted_center_y = center_y + shift_y
                global_shifted_center_x, global_shifted_center_y = int(shifted_center_x + x), int(shifted_center_y + y)
                global_shifted_center_x, global_shifted_center_y = self.global_coordinate_system.to_global(global_shifted_center_x, global_shifted_center_y)
                
                return (global_shifted_center_x, global_shifted_center_y)
        
        return (global_center_x, global_center_y)
