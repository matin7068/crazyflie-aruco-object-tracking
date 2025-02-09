import cv2
import zmq
import numpy as np
import struct

# Function to detect and send pose (yellow object and ArUco markers)
def detect_and_send_pose(cap, dictionary, parameters, camera_matrix, dist_coeffs, socket):
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect yellow color
        lower_yellow = np.array([20, 100, 100])  # Lower bound for yellow color in HSV
        upper_yellow = np.array([40, 255, 255])  # Upper bound for yellow color in HSV
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Noise removal using morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Bitwise AND to isolate the yellow areas
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours of the detected yellow areas
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centroid_x, centroid_y, depth_value = None, None, None

        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])

                # Draw the centroid on the image
                cv2.circle(frame, (cX, cY), 7, (0, 255, 0), -1)
                cv2.putText(frame, f"({cX}, {cY})", (cX + 10, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                centroid_x = cX
                centroid_y = cY
                depth_value = 100  # Simulated depth value

                pose_data = struct.pack('fff', centroid_x / 1000, centroid_y / 1000, depth_value / 100)
                socket.send(pose_data)

                print(f"Published raw bytes: {pose_data}")
                print(f"Published yellow pose: x={centroid_x / 1000}, y={centroid_y / 1000}, z={depth_value / 100}")

        # ArUco marker detection
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        if marker_ids is not None:
            frame = cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.063, camera_matrix, dist_coeffs)

            for i, marker_id in enumerate(marker_ids.flatten()):
                tvec = tvecs[i].reshape(3)

                # Send the pose data of ArUco marker
                pose_data = struct.pack('fff', tvec[0], tvec[1], tvec[2])
                socket.send(pose_data)

                print(f"Published raw bytes: {pose_data}")
                print(f"Published ArUco pose: x={tvec[0]}, y={tvec[1]}, z={tvec[2]}")

                # Draw axes for ArUco marker visualization
                axis = np.float32([[0.05, 0, 0], [0, 0.05, 0], [0, 0, -0.05], [0, 0, 0]])
                imgpts, _ = cv2.projectPoints(axis, rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
                origin = tuple(imgpts[3].ravel().astype(int))
                frame = cv2.line(frame, origin, tuple(imgpts[0].ravel().astype(int)), (0, 0, 255), 2)  # X-axis
                frame = cv2.line(frame, origin, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 2)  # Y-axis
                frame = cv2.line(frame, origin, tuple(imgpts[2].ravel().astype(int)), (255, 0, 0), 2)  # Z-axis
        else:
            print("No ArUco markers detected.")

        # Show the updated frame with detections
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Camera calibration parameters (replace with your calibration data)
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# Initialize webcam and ArUco dictionary
cap = cv2.VideoCapture(0)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()

# Initialize ZeroMQ
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://localhost:5555")

# Start detection and communication loop
detect_and_send_pose(cap, dictionary, parameters, camera_matrix, dist_coeffs, socket)
