import cv2
import os

# ---------------- PATHS ----------------

script_dir = os.path.dirname(os.path.abspath(__file__))

calibrating_dir = os.path.join(script_dir, "calibrating")
image_dir = os.path.join(calibrating_dir, "images")

os.makedirs(image_dir, exist_ok=True)

# ---------------- CAMERA ----------------

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

if not cap.isOpened():
    print("Kunne ikke åpne kamera")
    exit()

success, img = cap.read()

if success:

    existing_images = [
        f for f in os.listdir(image_dir)
        if f.endswith(".png")
    ]

    num = len(existing_images)

    filename = os.path.join(image_dir, f"img{num}.png")

    cv2.imwrite(filename, img)

    print(f"Bilde lagret som: {filename}")

else:
    print("Failed to grab frame")

cap.release()