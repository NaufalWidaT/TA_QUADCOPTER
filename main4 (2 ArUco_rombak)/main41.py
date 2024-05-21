### GUSTI PARING DALANE
### Programnya pake Velocity tanpa duration, jajal sek
###

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
from cv2 import aruco
import numpy as np
import threading


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    time.sleep(5)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

def velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0):
        vehicle.send_mavlink(msg)
        time.sleep(1)
def deteksi_aruco(frame, aruco_dict, parameters):
    global ids
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

            #Tengah Frame
            fx = frame.shape[1] // 2
            fy = frame.shape[0] // 2

            #Jarak tengah frame dan tengah ArUco
            jfx = fx - cX
            jfy = fy - cY


            # Draw the ArUco marker ID and size on the frame
            cv2.putText(frame, f"ID: {max_marker_id[0]}", (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Size: {max_marker_size:.2f}", (topLeft[0], topLeft[1] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Print position based on cX and cY
            gerakqc(cX, cY, jfx, jfy)

    return frame

def gerakqc(cX, cY, distx, disty):
    x = distx * (1/320)
    y = disty * (1/240)
    #print("x:",x,"y:",y)
    print("cX:", cX,"cY:",cY)
    #print("Distance X:",distx, "DistanceY:",disty)
    #time.sleep(1)

    if cX > 320:
        print("Bergerak Maju" )
        #velocity(x,0,0)
    elif cX < 320:
        print("Bergerak Mundur" )
        #velocity(x,0,0)

    if cY > 240:
        print("Bergerak Kanan")
        #velocity(0,y,0)
    elif cY < 240:
        print("Bergerak Kiri" )
        #velocity(0,y,0)


    if cX <= 340 and cX >= 300 and cY <= 260 and cY >= 220 :
        print("Diatas ArUco, menurunkan Ketinggian")
        #velocity(0,0,0.3)


def camera():
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
        frame = deteksi_aruco(frame, aruco_dict, parameters)

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    vehicle = connect("COM15", baud=57600)
    print("connect")

    vehicle.mode = VehicleMode("GUIDED")

    arm_and_takeoff(4)
    velocity(0.2, 0, 0, 2)

    t1 = threading.Thread(target=gerakqc)
    t2 = threading.Thread(target=camera)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")
