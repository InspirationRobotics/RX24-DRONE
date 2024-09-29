import numpy as np
import cv2 as cv
import os

# Dictionary mapping ArUco marker types to their corresponding OpenCV constants
ARUCO_DICT = {
    "DICT_4X4_50": cv.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
}

# Function to display detected ArUco markers on the image
def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # Turning corners to points
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # Draw the bounding box
            cv.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            # Find the center of the ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            # Draw the ArUco marker ID on the image
            cv.putText(image, f"Marker ID: {markerID}", (topLeft[0], topLeft[1] + 20), cv.FONT_HERSHEY_SIMPLEX,
                       0.4, (0, 150, 255), 2)
            print(f"[Inference] ArUco marker ID: {markerID}")

    return image

# Load the image
image_path = "apriltag_1_with_background.png"  # Update this to your image path

if not os.path.exists(image_path):
    print("Error: The specified image path does not exist.")
    exit()

image = cv.imread(image_path)

if image is None:
    print("Error: Could not load image from file. Check the path.")
    exit()

# Resize the image
scale_percent = 30  # Percentage of original size (e.g., 30% smaller)
width = int(image.shape[1] * scale_percent / 100)
height = int(image.shape[0] * scale_percent / 100)
dim = (width, height)
resized_image = cv.resize(image, dim, interpolation=cv.INTER_AREA)

# Convert to grayscale
gray = cv.cvtColor(resized_image, cv.COLOR_BGR2GRAY)

# Iterate over all ArUco dictionaries
detected = False
for aruco_name, aruco_dict in ARUCO_DICT.items():
    print(f"Trying with dictionary: {aruco_name}")
    arucoDict = cv.aruco.getPredefinedDictionary(aruco_dict)
    arucoParams = cv.aruco.DetectorParameters()

    # Detect markers
    corners, ids, rejected = cv.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)

    # Check if any markers are detected
    if ids is not None:
        print(f"Markers detected with dictionary: {aruco_name}")
        detected_image = aruco_display(corners, ids, rejected, resized_image)
        detected = True
        break

if detected:
    # Display the image with detected markers
    cv.imshow("Detected Markers", detected_image)
    cv.waitKey(0)
    cv.destroyAllWindows()
else:
    print("No markers detected with any ArUco dictionary.")
