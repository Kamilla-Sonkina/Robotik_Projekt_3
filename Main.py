import cv2
import torch
import time
from ObjectDetector import ObjectDetector
from TopDownTransformer import TopDownTransformer
from EuclideanDistTracker import EuclideanDistTracker  # Import der Klasse

def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Verwende Gerät: {device}")

    model_path = r"C:\Users\lg\Desktop\runs\detect\train5\weights\best.pt"
    detector = ObjectDetector(model_path, device)
    transformer = TopDownTransformer()
    tracker = EuclideanDistTracker()  # Initialisierung des Trackers

    video_path = r"C:\Users\lg\Downloads\Film am 27.05.24 um 15.44.mov"
   
    cap = cv2.VideoCapture(video_path)

    frame_count = 0
    results_cache = []

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        if frame_count % 1 != 0:
            frame_count += 1
            continue

        current_time = time.time()

        frame = transformer.get_transformed_frame(frame)
        height, width = frame.shape[:2]

        frame_reduced = cv2.resize(frame, (width // 2, height // 2))
        results_cache = detector.detect_objects(frame_reduced)
        print(f"Ergebnisse auf Frame {frame_count}: {results_cache}")

        # Konvertiere die Ergebnisse in das Format (x, y, w, h, label)
        objects_rect = []
        for result in results_cache:
            if len(result) >= 6:
                x1, y1, x2, y2 = [int(coord * 2) for coord in result[:4]]
                w = x2 - x1
                h = y2 - y1
                label = 'Einhorn' if int(result[5]) == 1 else 'Katze'
                objects_rect.append((x1, y1, w, h, label))
        
        # Aktualisiere den Tracker
        tracked_objects = tracker.update([rect[:4] for rect in objects_rect], current_time)
        
        # Zeichne die verfolgten Objekte
        for i, (x, y, w, h, id) in enumerate(tracked_objects):
            label = objects_rect[i][4]  # Label aus den Objekterkennungsergebnissen
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f'ID: {id} {label}', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            speed = tracker.get_speed(id)
            cv2.putText(frame, f'Speed: {speed:.2f} cm/s', (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        frame_count += 1

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
