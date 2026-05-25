import numpy as np
import cv2 as cv
import glob
import pickle
import os

# ---------------- SETTINGS ----------------

chessboardSize = (7, 5)  # antall indre hjørner, ikke antall ruter
frameSize = (640, 480)

size_of_chessboard_squares_mm = 20

# ---------------- FIND CHESSBOARD CORNERS ----------------

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
objp = objp * size_of_chessboard_squares_mm

objpoints = []
imgpoints = []

images = glob.glob("images/*.png")

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

        #cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        #cv.imshow("Chessboard corners", img)
        #cv.waitKey(500)

cv.destroyAllWindows()

print("Gyldige kalibreringsbilder:", len(objpoints))

if len(objpoints) == 0:
    print("Fant ingen gyldige sjakkbrettbilder.")
    print("Sjekk at:")
    print("- bildene ligger i images/")
    print("- chessboardSize stemmer med antall indre hjørner")
    print("- hele sjakkbrettet er synlig i bildene")
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

with open("calibration.pkl", "wb") as f:
    pickle.dump((cameraMatrix, dist), f)

with open("cameraMatrix.pkl", "wb") as f:
    pickle.dump(cameraMatrix, f)

with open("dist.pkl", "wb") as f:
    pickle.dump(dist, f)

print("\nKalibrering lagret.")

# ---------------- UNDISTORT TEST IMAGE ----------------

os.makedirs("calibrated_images", exist_ok=True)

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
    cv.imwrite(f"calibrated_images/calibrated_{filename}", dst)

print("Alle kalibrerte bilder er lagret i calibrated_images/")



test_images = glob.glob("images/*.png")

if len(test_images) > 0:
    img = cv.imread(test_images[0])

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

    cv.imwrite("caliResult1.png", dst)
    print("Undistorted test image saved as caliResult1.png")

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