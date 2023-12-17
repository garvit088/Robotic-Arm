import cv2
# from apriltag import apriltag
from pupil_apriltags import Detector
from pyzbar.pyzbar import decode
import socket
import pickle
import struct

# Function to detect AprilTag
def detect_apriltag(frame):
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detectedBarcodes = decode(frame)
    if detectedBarcodes:
        for barcode in detectedBarcodes:
            (x, y, w, h) = barcode.rect
        return detectedBarcodes
    else:
        return None

# Function to detect AprilTag
def detect_apriltag(frame):
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
    )
    result = at_detector.detect(frame)
    print(result)
    # detector = apriltag("tagStandard41h12")
    # result = detector.detect(gray)
    return result

# Camera setup
cap_overhead = cv2.VideoCapture(0)  # Adjust the camera index as needed
cap_diagonal1 = cv2.VideoCapture(1)  # Adjust the camera index as needed
cap_diagonal2 = cv2.VideoCapture(2)  # Adjust the camera index as needed

# # Socket setup
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind(("0.0.0.0", 5555))
# server_socket.listen(10)

# print("Waiting for client connection...")
# client_socket, addr = server_socket.accept()
# print("Client connected")

while True:
    # Capture frames from the cameras
    _, frame_overhead = cap_overhead.read()
    _, frame_diagonal1 = cap_diagonal1.read()
    _, frame_diagonal2 = cap_diagonal2.read()

    # Detect AprilTag on each frame
    result_overhead = detect_apriltag(frame_overhead)
    result_diagonal1 = detect_apriltag(frame_diagonal1)
    result_diagonal2 = detect_apriltag(frame_diagonal2)

    # Determine which face has the AprilTag
    if result_overhead:
        face = "Overhead"
    elif result_diagonal1:
        face = "Diagonal1"
    elif result_diagonal2:
        face = "Diagonal2"
    else:
        face = "None"

    # Send face information to the client
    # data = pickle.dumps(face)
    # client_socket.sendall(struct.pack("I", len(data)) + data)

# Release resources
cap_overhead.release()
cap_diagonal1.release()
cap_diagonal2.release()
# server_socket.close()
