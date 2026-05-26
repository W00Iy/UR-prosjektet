import cv2
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
image_dir = os.path.join(script_dir, "images")

os.makedirs(image_dir, exist_ok=True)

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

if not cap.isOpened():
    print("Kunne ikke åpne kamera")
    exit()

success, img = cap.read()

if success:
    num = len(os.listdir(image_dir))
    filename = os.path.join(image_dir, f"img{num}.png")

    cv2.imwrite(filename, img)
    print(f"Bilde lagret som {filename}")
else:
    print("Failed to grab frame")

cap.release()