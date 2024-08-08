import cv2

cap = cv2.VideoCapture('/dev/video0')  # 0 for the primary webcam
cap.set(3, 640)  # Width
cap.set(4, 480)  # Height

while True:
    success, img = cap.read()
    if not success:
        break
    cv2.imshow("Webcam Output", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
