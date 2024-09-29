import cv2 as cv
from cv2 import aruco
import numpy as np
import time

from drone_API import drone_API, RCLPY_Handler

calib_data_path = "calib_data/MultiMatrix.npz"

# Gotta import the distortion to accurately record 3D position
calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"] # camera matrix to transform 3D point to 2D
dist_coef = calib_data["distCoef"] # Lens distortion
r_vectors = calib_data["rVector"] # rotation
t_vectors = calib_data["tVector"] # position

MARKER_SIZE_BIG = 120  # centimeters
MARKER_SIZE_SMALL = 20.3
HEIGHT_FOR_CHANGE = 152.4 # 5 ft

marker_dict_big = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11) # loads the april tag fam
marker_dict_small = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)

param_markers = aruco.DetectorParameters()

cap = cv.VideoCapture(1)

handler = RCLPY_Handler("drone_API_node")
drone = drone_API(handler)
drone.connect()

time.sleep(2)

drone.arm()

'''def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6  # If the matrix is singular

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # Roll
        y = np.arctan2(-R[2, 0], sy)  # Pitch
        z = np.arctan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])  # Roll
        y = np.arctan2(-R[2, 0], sy)  # Pitch
        z = 0  # Yaw

    return np.degrees(x), np.degrees(y), np.degrees(z)  # Convert to degrees
'''
def aruco_detection_landing():
    while True:
        ret, frame = cap.read()
        if not ret: # if not open camera
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # converts to gray scale for detectMarkers
        # detect markers in grayscale
        marker_corners_big, marker_IDs_big, reject = aruco.detectMarkers(
            gray_frame, marker_dict_big, parameters=param_markers
        )

        # marker corners = 2D coords of corners in image
        # marker_Ids = id of detected marker

        if marker_corners_big is not None:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners_big, MARKER_SIZE_BIG, cam_mat, dist_coef
            )

            # rvec = rotation vector of marker relative to camm
            # tvec = translation vector indicates markers position in 3D

            #total_markers = range(0, marker_IDs.size)
            for i, corners in enumerate(marker_corners_big):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )

                # polylines draws the corners of detected marker (square around marker)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel().astype(int)
                top_left = corners[1].ravel().astype(int)
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Calculating the distance
                '''distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )'''

                offset_x = tVec[0][0][0]
                offset_y = tVec[0][0][1]
                offset_z = tVec[0][0][2]

                drone.set_velocity(-offset_x*0.1, -offset_y*0.1, -offset_z*0.1)

                if abs(offset_z) < 600 and abs(offset_x) < 10 and abs(offset_y) < 10:

                    cv.putText(frame, f"Z: {round(z_height_big, 2)} cm", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                    z_height_big = tVec[0][0][2]
                    
                    if z_height_big < HEIGHT_FOR_CHANGE:

                        marker_corners_small, marker_IDs_small, reject = aruco.detectMarkers(
                        gray_frame, marker_dict_small, parameters=param_markers
                        )

                        if marker_corners_small is not None and len(marker_corners_small) >= 2:
                            print("smalll markers detected")
                            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                            marker_corners_small, MARKER_SIZE_SMALL, cam_mat, dist_coef
                            )

                            for i, corners in enumerate(marker_corners_small):
                                cv.polylines(
                                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                                )
                                
                                corners = corners.reshape(4, 2)
                                corners = corners.astype(int)
                                top_right = corners[0].ravel().astype(int)
                                top_left = corners[1].ravel().astype(int)
                                bottom_right = corners[2].ravel()
                                bottom_left = corners[3].ravel()

                                point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

                                offset_x = np.mean(tVec[1][0][0], tVec[2][0][0])
                                offset_y = np.mean(tVec[1][0][1], tVec[2][0][1])
                                offset_z = np.mean(tVec[1][0][2], tVec[2][0][2])

                                drone.set_velocity(-offset_x*0.1, -offset_y*0.1, -offset_z*0.1)

                                if abs(offset_z) < 10 and abs(offset_x) < 5 and abs(offset_y) < 5:
                                    drone.land()
                                    break

                                cv.circle(frame, (offset_x, offset_y), 5, (0, 0, 255), -1)

                                text_position_x = 10
                                text_position_y = 30

                                cv.putText(frame, f"X: {round(offset_x, 2)} cm", 
                                    (text_position_x,text_position_y + 20), cv.FONT_HERSHEY_SIMPLEX, 
                                    0.6, (0, 0, 255), 2, cv.LINE_AA)  # X-axis Red
                                
                                cv.putText(frame, f"Y: {round(offset_y, 2)} cm", 
                                    (text_position_x, text_position_y + 40), cv.FONT_HERSHEY_SIMPLEX, 
                                    0.6, (0, 255, 0), 2, cv.LINE_AA)  # Y-axis Green
                                
                                cv.putText(frame, f"Z: {round(offset_z, 2)} cm", 
                                    (text_position_x, text_position_y + 60), cv.FONT_HERSHEY_SIMPLEX, 
                                    0.6, (0, 255, 0), 2, cv.LINE_AA)  # Y-axis Green

                # Convert rotation vector to rotation matrix
                '''rotation_matrix, _ = cv.Rodrigues(rVec[i])

                # Calculate Euler angles (yaw, pitch, roll)
                roll, pitch, yaw = rotationMatrixToEulerAngles(rotation_matrix)'''

                # Draws the pose of the marker
                '''point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

                # Text for distances and orientations
                cv.putText(frame, f"ID: {ids[0]} Z: {round(tVec[i][0][2], 2)} cm", top_right, 
                        cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv.LINE_AA)  # Z-axis Blue
                cv.putText(frame, f"X: {round(tVec[i][0][0], 2)} cm", 
                        (top_right[0], top_right[1] + 20), cv.FONT_HERSHEY_SIMPLEX, 
                        0.6, (0, 0, 255), 2, cv.LINE_AA)  # X-axis Red
                cv.putText(frame, f"Y: {round(tVec[i][0][1], 2)} cm", 
                        (top_right[0], top_right[1] + 40), cv.FONT_HERSHEY_SIMPLEX, 
                        0.6, (0, 255, 0), 2, cv.LINE_AA)  # Y-axis Green'''

                '''cv.putText(frame, f"Yaw: {round(yaw, 2)}", 
                        (top_right[0] + 200, top_right[1] + 0), cv.FONT_HERSHEY_SIMPLEX, 
                        0.6, (255, 0, 0), 2, cv.LINE_AA)  # Yaw Blue
                cv.putText(frame, f"Pitch: {round(pitch, 2)}", 
                        (top_right[0] + 150, top_right[1] + 40), cv.FONT_HERSHEY_SIMPLEX, 
                        0.6, (0, 255, 0), 2, cv.LINE_AA)  # Pitch Green
                cv.putText(frame, f"Roll: {round(roll, 2)}", 
                        (top_right[0] + 150, top_right[1] + 20), cv.FONT_HERSHEY_SIMPLEX, 
                        0.6, (0, 0, 255), 2, cv.LINE_AA)  # Roll Red'''

        cv.imshow("frame", frame)
        key = cv.waitKey(1)

        # In order to turn off the recording, PRESS Q, which then closes all windows
        if key == ord("q"):
            break
    cap.release()
    cv.destroyAllWindows()

aruco_detection_landing()
drone.disconnect()