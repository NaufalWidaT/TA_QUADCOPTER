from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
from cv2 import aruco
import numpy as np

a = 0

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

def yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an abs
        # olute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        30,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

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
    vehicle.send_mavlink(msg)

def deteksi_aruco(frame, aruco_dict, parameters):
    global a
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Variable to store the size and corners of the largest marker
    max_marker_size = 0
    max_marker_corners = None
    max_marker_id = None

    if ids is not None and len(ids) > 0:
        # Draw detected markers
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

            # Tengah Frame
            fx = frame.shape[1] // 2
            fy = frame.shape[0] // 2

            # Jarak tengah frame dan tengah ArUco
            jfx = fx - cX
            jfy = fy - cY

            # Draw the ArUco marker ID and size on the frame
            cv2.putText(frame, f"ID: {max_marker_id[0]}", (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
            cv2.putText(frame, f"Size: {max_marker_size:.2f}", (topLeft[0], topLeft[1] - 35), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

            # Print position based on cX and cY
            gerakqc(cX, cY, jfx, jfy)

    else:
        # If no markers detected
        print("No ArUco markers detected.")
        a += 1
        print(a)
        yaw(90)
        if a == 150:
            print("ArUco Tidak ditemukan, Landing")
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            exit(1)

    return frame


def gerakqc(cX, cY, distx, disty):
    ## Hitung kecepatan
    x = (distx * (1 / 320))/1.5
    y = (disty * (1 / 240))/1.5

    print("speedx:", x, "speedy:", y, "cX:", cX, "cY:", cY)
    print(vehicle.location.global_relative_frame.alt)

    if cX <= 290 and cX >= 0:
        print("Bergerak Maju")
        velocity(x,0,0)
    elif cX >= 350 and cX <= 640:
        print("Bergerak Mundur")
        velocity(x,0,0)
    elif cY >= 270 and cY <= 480:
        print("Bergerak Kiri")
        velocity(0,y,0)
    elif cY <= 210 and cY >= 0:
        print("Bergerak Kanan")
        velocity(0,y,0)

    if cX <= 350 and cX >= 290 and cY <= 270 and cY >= 210:
        print("Diatas ArUco, menurunkan Ketinggian")
        velocity(0,0,0.4)

########################################## LANDING #####################################################

    if cX <= 350 and cX >= 290 and cY <= 270 and cY >= 210 and vehicle.location.global_relative_frame.alt <= 1.5:
        print("Diatas ArUco, Landing")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
        exit(1)

#Konek ke QC
vehicle = connect("COM15", baud=57600)
print("connect")

#QC mode guided
vehicle.mode = VehicleMode("GUIDED")

#Take off
arm_and_takeoff(4)

#Maju ke depan awal2
print("Maju awal")
velocity(0.2, 0, 0)

# Load the dictionary that was used to generate the markers.
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters using default values
parameters = cv2.aruco.DetectorParameters()

# Capture video from the first webcam on your system
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture image")
        break

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
