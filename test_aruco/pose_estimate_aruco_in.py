import cv2 as cv
from cv2 import aruco
import numpy as np

calib_data_path = "calib_data/MultiMatrix.npz"

# Load camera calibration data
calib_data = np.load(calib_data_path)

cam_mat = calib_data["camMatrix"] # camera matrix
dist_coef = calib_data["distCoef"] # lens distortion

MARKER_SIZE = 20  # centimeters change as needed
pixels_per_cm = 170 / 20.3  # approx 8.37 pixels per cm

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11) # AprilTag dictionary
param_markers = aruco.DetectorParameters()

cap = cv.VideoCapture(1)

alpha = 1.0  # Adjust as needed
beta = 0  # Adjust as needed

# Initialize lists to store previous pose vectors for smoothing
rVecs_hist = []
tVecs_hist = []
HIST_SIZE = 5  # Number of frames to average over

# Create CLAHE object for contrast enhancement
clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv.addWeighted(frame, alpha, np.zeros(frame.shape, frame.dtype), 0, beta)
    # Convert to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Apply CLAHE to improve contrast
    gray_frame = clahe.apply(gray_frame)

    # Detect markers
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_corners:
        # Estimate pose of detected markers
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )

        # Calculate the center of the frame
        frame_center_x = int(frame.shape[1] / 2)
        frame_center_y = int(frame.shape[0] / 2)



        # Append the current vectors to history and limit to HIST_SIZE frames
        rVecs_hist.append(rVec)
        tVecs_hist.append(tVec)
        if len(rVecs_hist) > HIST_SIZE:
            rVecs_hist.pop(0)
            tVecs_hist.pop(0)

        # Compute average pose vectors for smoother results
        avg_rVec = np.mean(np.array(rVecs_hist), axis=0)
        avg_tVec = np.mean(np.array(tVecs_hist), axis=0)

        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            # Draw marker corners
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )

            # Draw pose axes using averaged pose estimation
            cv.drawFrameAxes(frame, cam_mat, dist_coef, avg_rVec[i], avg_tVec[i], 4, 4)

            # Display marker information
            corners = corners.reshape(4, 2).astype(int)
            top_right = corners[0].ravel()

            (cX, cY) = (int(corners[:, 0].mean()), int(corners[:, 1].mean()))
            
            # Calculate the offset from the center of the frame
            offset_x = cX - frame_center_x
            offset_y = cY - frame_center_y

            offset_x_cm = offset_x / pixels_per_cm
            offset_y_cm = offset_y / pixels_per_cm

            # Print out the offsets for comparison
            print(f"Marker ID: {ids[0]} Offset X: {offset_x_cm}, Offset Y: {offset_y_cm}")

            # Display position info
            cv.putText(frame, f"ID: {ids[0]} Z: {round(avg_tVec[i][0][2], 2)} cm", top_right, 
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv.LINE_AA)
            cv.putText(frame, f"X: {round(avg_tVec[i][0][0], 2)} cm", 
                       (top_right[0], top_right[1] + 20), cv.FONT_HERSHEY_SIMPLEX, 
                       0.6, (0, 0, 255), 2, cv.LINE_AA)
            cv.putText(frame, f"Y: {round(avg_tVec[i][0][1], 2)} cm", 
                       (top_right[0], top_right[1] + 40), cv.FONT_HERSHEY_SIMPLEX, 
                       0.6, (0, 255, 0), 2, cv.LINE_AA)
    
    '''corner_A = corners[0][0]  # Top-left corner
    corner_B = corners[0][1]  # Top-right corner
    marker_size_in_pixels = np.linalg.norm(corner_A - corner_B)
    print(marker_size_in_pixels)'''
    
    # Display the frame
    cv.imshow("frame", frame)
    
    # Exit on 'q' press
    key = cv.waitKey(1)
    if key == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
