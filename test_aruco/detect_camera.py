import cv2

def check_cameras():
    
    for index in range(0,13):
        cap = cv2.VideoCapture(index)
        if not cap.read()[0]:
            print(f"Camera with index {index} is not available.")
            cap.release()
            
        else:
            print(f"Camera with index {index} is available.")
        cap.release()
    
check_cameras()
