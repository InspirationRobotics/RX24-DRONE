import cv2 as cv
import os

CHESS_BOARD_DIM = (8, 5)

n = 0  # image counter

# Check if images directory exists; if not, create it
image_dir_path = "images"
CHECK_DIR = os.path.isdir(image_dir_path)

if not CHECK_DIR:
    os.makedirs(image_dir_path)
    print(f'"{image_dir_path}" Directory is created')
else:
    print(f'"{image_dir_path}" Directory already exists.')

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def detect_checker_board(image, grayImage, criteria, boardDimension):
    # Attempt to find chessboard corners
    ret, corners = cv.findChessboardCorners(grayImage, boardDimension)

    if ret:  # If chessboard corners are found
        corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)
        return image, True  # Return the modified image and True if corners are detected
    else:
        # Return the original image and False if no corners are detected
        return image, False

cap = cv.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image from the camera.")
        break

    copyFrame = frame.copy()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Call the updated function to detect chessboard corners
    image, board_detected = detect_checker_board(frame, gray, criteria, CHESS_BOARD_DIM)

    # Display the number of saved images
    cv.putText(
        frame,
        f"saved_img : {n}",
        (30, 40),
        cv.FONT_HERSHEY_PLAIN,
        1.4,
        (0, 255, 0),
        2,
        cv.LINE_AA,
    )

    cv.imshow("frame", frame)
    cv.imshow("copyFrame", copyFrame)

    key = cv.waitKey(1)

    if key == ord("q"):
        break

    # Save the image if 's' is pressed and a board is detected
    if key == ord("s") and board_detected:
        if n < 30:
            cv.imwrite(f"{image_dir_path}/image{n}.png", copyFrame)
            print(f"Saved image number {n}")
            n += 1  # Increment the image counter
        else:
            print("Reached the limit of 30 images.")
            break  # Exit the loop after saving 30 images

cap.release()
cv.destroyAllWindows()

print("Total saved images:", n)
