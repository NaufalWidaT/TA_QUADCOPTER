import cv2
import numpy as np

def detect_largest_aruco_marker(frame, aruco_dict, parameters):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Variable to store the size and corners of the largest marker
    max_marker_size = 0
    max_marker_corners = None
    max_marker_id = None

    # Draw detected markers
    if ids is not None:
        for markerCorner, markerID in zip(corners, ids):
            # Extract the marker corners
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # Convert each of the (x, y)-coordinate pairs to integers
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

            # Convert the points to numpy arrays
            topLeft_np = np.array(topLeft)
            topRight_np = np.array(topRight)
            bottomLeft_np = np.array(bottomLeft)
            bottomRight_np = np.array(bottomRight)

            # Calculate the size of the marker
            width = np.linalg.norm(topRight_np - topLeft_np)
            height = np.linalg.norm(bottomLeft_np - topLeft_np)
            size = (width + height) / 2  # Average the width and height for size estimate

            # Check if this marker is the largest so far
            if size > max_marker_size:
                max_marker_size = size
                max_marker_corners = markerCorner
                max_marker_id = markerID

        if max_marker_corners is not None:
            # Extract the largest marker corners
            corners = max_marker_corners.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # Convert each of the (x, y)-coordinate pairs to integers
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))

            # Draw the bounding box of the largest ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

            # Compute and draw the center (x, y)-coordinates of the ArUCo marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

            # Draw the ArUco marker ID and size on the frame
            cv2.putText(frame, f"ID: {max_marker_id[0]}", (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Size: {max_marker_size:.2f}", (topLeft[0], topLeft[1] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Print position based on cX and cY
            print_position(cX, cY)

    return frame

def print_position(cX, cY):
    if cX > 320:
        print("cX is greater than 320")
    elif cX < 320:
        print("cX is less than 320")

    if cY > 240:
        print("cY is greater than 240")
    elif cY < 240:
        print("cY is less than 240")

def main():
    # Load the dictionary that was used to generate the markers.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Initialize the detector parameters using default values
    parameters = cv2.aruco.DetectorParameters()

    # Capture video from the first webcam on your system
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Detect and display the largest Aruco marker
        frame = detect_largest_aruco_marker(frame, aruco_dict, parameters)

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
