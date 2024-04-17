import cv2
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def hand_detection(image):
    # Hier wird die Handerkennung durchgeführt
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands()
    
    # Konvertiere das Frame zu RGB (Mediapipe benötigt RGB-Format)
    rgb_frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Erkenne Hände im Frame
    results = hands.process(rgb_frame)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Extrahiere die Koordinaten der Handlandmarken
            landmark_points = []
            for landmark in hand_landmarks.landmark:
                x = int(landmark.x * image.shape[1])
                y = int(landmark.y * image.shape[0])
                landmark_points.append((x, y))

            # Konstruiere das Begrenzungsrechteck um die Handlandmarken
            if landmark_points:
                points = np.array(landmark_points, dtype=np.int32)
                bounding_box = cv2.boundingRect(points)
                x, y, w, h = bounding_box
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Zeige das verarbeitete Frame an
    cv2.imshow("Hand Detection", image)
    cv2.waitKey(1)

def image_callback(msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    hand_detection(frame)

def main():
    # Starte den ROS-Knoten
    rospy.init_node('hand_detection_node', anonymous=True)

    # Initialisiere den ROS-Bildveröffentlichungsknoten
    rospy.Subscriber("/camera/image_raw", Image, image_callback)

    # Öffne die Videokamera
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # Lese das Frame von der Kamera
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to read frame")
            break

        # Rufe die Funktion zur Handerkennung auf
        hand_detection(frame)

    # Freigabe der Ressourcen
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
