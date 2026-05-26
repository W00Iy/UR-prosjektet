import numpy as np
import cv2 as cv
import glob
import pickle
import os
import shutil

# ---------------- PATHS ----------------

script_dir = os.path.dirname(os.path.abspath(__file__))

calibrating_dir = os.path.join(script_dir, "calibrating")
image_dir = os.path.join(calibrating_dir, "images")
calibrated_dir = os.path.join(calibrating_dir, "calibrated_images")

os.makedirs(image_dir, exist_ok=True)
os.makedirs(calibrated_dir, exist_ok=True)

# ---------------- DELETE OLD CALIBRATION FILES ----------------

old_files = [
    "calibration.pkl",
    "cameraMatrix.pkl",
    "dist.pkl",
    "caliResult1.png"
]

for file in old_files:
    path = os.path.join(calibrating_dir, file)

    if os.path.exists(path):
        os.remove(path)
        print(f"Slettet gammel fil: {file}")

# ---------------- SETTINGS ----------------

chessboardSize = (7, 5)
frameSize = (640, 480)
size_of_chessboard_squares_mm = 20

# ---------------- FIND CHESSBOARD CORNERS ----------------

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
objp = objp * size_of_chessboard_squares_mm

objpoints = []
imgpoints = []

images = glob.glob(os.path.join(image_dir, "*.png"))

print("Bilder funnet:", len(images))

for image in images:
    img = cv.imread(image)

    if img is None:
        print("Kunne ikke lese bilde:", image)
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    print(image, "sjakkbrett funnet:", ret)

    if ret:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria
        )

        imgpoints.append(corners2)

cv.destroyAllWindows()

print("Gyldige kalibreringsbilder:", len(objpoints))

if len(objpoints) == 0:
    print("Fant ingen gyldige sjakkbrettbilder.")
    print("Bildene må ligge i:", image_dir)
    exit()

# ---------------- CALIBRATION ----------------

ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints,
    imgpoints,
    frameSize,
    None,
    None
)

print("\nCamera matrix:")
print(cameraMatrix)

print("\nDistortion coefficients:")
print(dist)

# ---------------- SAVE CALIBRATION ----------------

with open(os.path.join(calibrating_dir, "calibration.pkl"), "wb") as f:
    pickle.dump((cameraMatrix, dist), f)

with open(os.path.join(calibrating_dir, "cameraMatrix.pkl"), "wb") as f:
    pickle.dump(cameraMatrix, f)

with open(os.path.join(calibrating_dir, "dist.pkl"), "wb") as f:
    pickle.dump(dist, f)

print("\nKalibrering lagret i:", calibrating_dir)

# ---------------- UNDISTORT IMAGES ----------------

for image in images:
    img = cv.imread(image)
    h, w = img.shape[:2]

    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(
        cameraMatrix, dist, (w, h), 1, (w, h)
    )

    dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    filename = os.path.basename(image)
    output_path = os.path.join(calibrated_dir, f"calibrated_{filename}")

    cv.imwrite(output_path, dst)

print("Kalibrerte bilder lagret i:", calibrated_dir)

# ---------------- TEST IMAGE ----------------

if len(images) > 0:
    img = cv.imread(images[0])
    h, w = img.shape[:2]

    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(
        cameraMatrix,
        dist,
        (w, h),
        1,
        (w, h)
    )

    dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    cv.imwrite(os.path.join(calibrating_dir, "caliResult1.png"), dst)

# ---------------- REPROJECTION ERROR ----------------

mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(
        objpoints[i],
        rvecs[i],
        tvecs[i],
        cameraMatrix,
        dist
    )

    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    mean_error += error

print("Total reprojection error:", mean_error / len(objpoints))

# ---------------- CLEANUP ----------------

print("Sletter bilder etter kalibrering...")

shutil.rmtree(image_dir, ignore_errors=True)
shutil.rmtree(calibrated_dir, ignore_errors=True)

print("Ferdig.")