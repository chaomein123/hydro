import cv2
import requests
import time
import threading

def capture_and_send_frame():
    try:
        # Open the camera
        cap = cv2.VideoCapture(0)  # Change the index if you have multiple cameras
        if not cap.isOpened():
            print("Failed to open the camera")
            return

        while True:
            # Capture a frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame")
                break

            # Encode the frame as JPEG
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                print("Failed to encode frame")
                break

            try:
                # Send the frame to the API
                files = {'file': ('frame.jpg', jpeg.tobytes(), 'image/jpeg')}
                response = requests.post('https://object-detection-server-8irf.onrender.com/detect', files=files)
                print('Data sent to API, response:', response.status_code)
            except requests.exceptions.RequestException as e:
                print(f"Failed to send data to API: {e}")

            # Sleep for 4 hours
            time.sleep(20)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Release the camera when done
        cap.release()