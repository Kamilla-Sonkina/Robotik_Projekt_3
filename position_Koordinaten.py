import cv2
import numpy as np

def transform_perspective(frame, src_points, dst_points): # Funktion mit den Parameter, frame list Bild ein, Koordinaten für Transformation
    
    matrix = cv2.getPerspectiveTransform(src_points, dst_points) # Transformationsmatrix berechnen, 4 Punkte werden im ursprung 
                                                                # in ziel Koordinaten Transformiert 
    
    result_frame = cv2.warpPerspective(frame, matrix, (frame.shape[1], frame.shape[0]))  # Das transformierte bild wird auf den frame angewendet
                                                                                        # Matrix wurde durch Transform berechnet 
                                                                                        # Tupel def. größe 1: breite 0: höhe
                                                                                        # hier ausgangsbild gleiche wie eingangsbild 
    return result_frame

def draw_coordinate_system(frame, origin, x_axis_point): # origin: koordinaten 0, x: länge x-Achse 
    
    frame_with_coordinate_system = frame.copy() # Bild Kopie, um das Koordinatensystem zu zeichnen
    
    cv2.arrowedLine(frame_with_coordinate_system, origin, x_axis_point, (0, 0, 255), 2) # Zeichne die x-Achse (GRB) 2: dicke
   
    cv2.arrowedLine(frame_with_coordinate_system, origin, (origin[0], 735), (0, 0, 255), 2)  # y-Achse zum linken oberen Punkt (415, 735)
   
    cv2.putText(frame_with_coordinate_system, '0', (origin[0] + 10, origin[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) # Beschriftung 
                                                                                            # auf kopiertes Frame, 0: bezeichnung,
                                                                                            # position der beschriftung 
                                                                                            # font typ von CV, 1: gröse, frabe, 2: dicke
    # Beschrifte die x-Achse
    cv2.putText(frame_with_coordinate_system, 'X', (x_axis_point[0] - 20, x_axis_point[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    # Beschrifte die y-Achse
    cv2.putText(frame_with_coordinate_system, 'Y', (origin[0] + 10, 750), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return frame_with_coordinate_system

# Punkte der Perspektivtransformation
src_points = np.float32([[415, 735], [1510, 760], [405, 975], [1500, 960]])  # Punkte im Ausgangsbild, float32: NumPy funktion 
                                                                                # Wandelt liste von koordinaten in array um 
                                                                                # jeder punkt ist lisste mit 2 elementen
dst_points = np.float32([[415, 735], [1500, 735], [415, 975], [1500, 975]])  # Punkte im Zielbild


video_path = r"/Users/johanneszotter/Downloads/Aufzeichnung 2024-04-25 110816.mp4"
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Lese die Höhe und Breite des Videos aus
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) # breite video, .get gibt dann angeforderten wert zurück
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) # höhe video

# Benenne das Fenster und passe die Größe an
cv2.namedWindow('Transformed Frame with Object Detection', cv2.WINDOW_NORMAL) # bezeichnung und scaliert bild in kameraauflöung 
cv2.resizeWindow('Transformed Frame with Object Detection', width, height) # hier kann eigensdefinierte gröse des fesnters angegeben werden

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Transformiere das Bild in die Vogelperspektive
    transformed_frame = transform_perspective(frame, src_points, dst_points) # funktion, frame einlesen, einlesen der variablen (array punke)
    
    # Zeichne das Koordinatensystem auf das transformierte Bild
    frame_with_coordinate_system = draw_coordinate_system(transformed_frame, (415, 975), (1500, 975)) # koordinatensystem zeichnen 
    
    # Konvertiere das Bild zurück zu BGR (von RGB)
    frame_with_coordinate_system = cv2.cvtColor(frame_with_coordinate_system, cv2.COLOR_RGB2BGR) # 
    
    # Konvertiere das Bild zu HSV
    hsv = cv2.cvtColor(frame_with_coordinate_system, cv2.COLOR_BGR2HSV) 
    
    # Definiere den Farbbereich für Weiß
    lower_white = np.array([0, 0, 100]) # ab wann weis erkannt wird
    upper_white = np.array([180, 50, 255]) # bis wo weiß erkannt wird
    
    # Erstelle eine Maske für weiße Objekte
    mask = cv2.inRange(hsv, lower_white, upper_white) # 
    
    # Finde Konturen der weißen Objekte
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Zeichne Umrandungen um die weißen Objekte innerhalb des Bereichs
    if len(contours) > 0:
        for contour in contours:
            # Berechne das umgebende Rechteck
            x, y, w, h = cv2.boundingRect(contour)
            # Überprüfe, ob das Rechteck innerhalb des umrahmten Bereichs liegt
            if x > dst_points[0][0] and y > dst_points[0][1] and x + w < (dst_points[3][0]) and y + h < (dst_points[3][1]):
                # Zeichne das Rechteck um das gefundene Objekt
                cv2.rectangle(frame_with_coordinate_system, (x, y), (x + w, y + h), (0, 255, 0), 2) # neu berreich ausschneiden 
                # Berechne die Position des Objekts relativ zum Koordinatensystem
                relative_position = (x - 415, 975 - y) # Differenz zwischen Objektposition und Ursprung des Koordinatensystems
                # Ausgabe der relativen Position
                cv2.putText(frame_with_coordinate_system, f'Relative Position: {relative_position}', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                # Zeige das transformierte Bild mit Umrandungen und Koordinatensystem an
                cv2.imshow('Transformed Frame with Object Detection', frame_with_coordinate_system)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
