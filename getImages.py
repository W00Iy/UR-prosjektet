import cv2
import os

os.makedirs("images", exist_ok=True)

cap = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)

if not cap.isOpened():
    print("Kunne ikke åpne kamera")
    exit()

num = 0

while True:
    success, img = cap.read()

    if not success:
        print("Failed to grab frame")
        break

    cv2.imshow("Img", img)

    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord("s"):
        cv2.imwrite(f"images/img{num}.png", img)
        print(f"Image {num} saved")
        num += 1

cap.release()
cv2.destroyAllWindows()