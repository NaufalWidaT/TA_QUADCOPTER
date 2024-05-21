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

def velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def cari_aruco():
    global x, y, z, id, cx, cy, titik
    while True:
        global a
        a = 0
#If Aruconya masih jauh
#        if id == 5 and x<=40 and x>=20 and y<=40 and y>=20 :
#            print("ID = ", id, "Terbaca, bergerak mundur kiri")
#            velocity(-0.2, -0.2, 0, 1)

#        elif id == 5 and x>=-40 and x<=-5 and y>=-30 and y<=-5 :
#            print("ID = ", id, "Terbaca, bergerak maju kanan")
#            velocity(0.2, 0.2, 0, 1)

#        elif id == 5 and x <= 40 and x >= 20 and y >= -30 and y <= -5:
#            print("ID = ", id, "Terbaca, bergerak mundur kanan")
#            velocity(-0.2, 0.2, 0, 1)

#        elif id == 5 and x >= -40 and x <= -5 and y <= 40 and y >= 20:
#            print("ID = ", id, "Terbaca, bergerak maju kiri")
#            velocity(0.2, -0.2, 0, 1)

#If Aruco jauh dan hanya di 1 sisi
        if id == 5 and z <= 500 and z >= 100 :

            print("x : ", cx, "y : ", cy)

            if cx <= 350 and cx >= 640:
                print("ID = ", id, "Terbaca, bergerak mundur")
                velocity(-0.2,0, 0, 1)
            elif cx >= 290 and cx <= 0:
                print("ID = ", id, "Terbaca, bergerak maju")
                velocity(0.2, 0, 0, 1)
            elif cy <= 210 and cy >= 0:
                print("ID = ", id, "Terbaca, bergerak kanan")
                velocity(0, 0.2, 0, 1)
            elif cy >= 270 and cy <= 480:
                print("ID = ", id, "Terbaca, bergerak kiri")
                velocity(0, -0.2, 0, 1)

# IF (x,y) ArUco masuk toleransi, tapi masih antara 200-400cm

            elif cx >= 300 and cx <= 340 and cy >= 220 and cy <= 260:
                print("Masuk toleransi, bergerak turun")
                velocity(0, 0, 0.3, 1)
#IF ArUco antara 100-160cm
#        if id == 5 and z <= 150:

#            print("x : ", x)
 #           print("y : ", y)

#            if x >= 4 :
 #               print("ID = ", id, "Terbaca dibawah 200cm, bergerak mundur")
  #              velocity(-0.1, 0, 0, 1)

   #         elif x <= 4 :
    #            print("ID = ", id, "Terbaca dibawah 200cm, bergerak maju")
     #           velocity(0.1, 0, 0, 1)

      #      elif y >= 5 :
       #         print("ID = ", id, "Terbaca dbawah 200cm, bergerak kanan")
        #        velocity(0, 0.1, 0, 1)

         #   elif y <= 5:
          #      print("ID = ", id, "Terbaca dibawah 200cm, bergerak kiri")
           #     velocity(0, -0.1, 0, 1)

#######################################  LANDING  ###################################################

        if id == 5 and cx >= 300 and cx <= 340 and cy >= 220 and cy <= 260 and z <=100 and z>=50:
            print("Diatas ArUco,Landing, A5")
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            exit(1)

        if id == 0 and z <= 100 and z >= 50  :
            print("Diatas ArUco,Landing, A0")
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            exit(1)

        for a in range(50):
            if id == 0 and z >= 100:
                print("ArUco tidak terdeteksi, Berputar mencari ArUco Marker")
                yaw(90)
                a += 1
                print(a)
                while a == 50:
                    time.sleep(1)
                    print("ArUco Tidak ditemukan, Landing")
                    vehicle.mode = VehicleMode("LAND")
                    vehicle.close()
                    exit(1)

def kamera_aruco():
    global id, cx, cy
    calib_data_path = "MultiMatrix.npz"
    calib_data = np.load(calib_data_path)
    print(calib_data.files)

    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]

    MARKER_SIZE = 8  # CM
    marker_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
    param_markers = cv2.aruco.DetectorParameters()
    cap = cv2.VideoCapture(0)

    while True:
        global id, z, titik
        ret, frame = cap.read()
        if not ret:
            break
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )

        if marker_corners:
            global rVec, tVec, ids, cx, cy
            rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)

            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[1].ravel()
                top_left = corners[0].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Nambah titik tengah di ArUco Marker
                cx = int((top_left[0] + bottom_right[0]) / 2)
                cy = int((top_left[1] + bottom_right[1]) / 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                #distance
                cv2.putText(
                    frame,
                    f"Dist: {2.95 * round(tVec[i][0][2], 2)}",
                    (0, 30),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.5,
                    (255, 25, 25),
                    2,
                    cv2.LINE_AA,
                )

                #nilai x
                cv2.putText(
                    frame,
                    f"x:{cx}",
                    (0, 65),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.5,
                    (5, 255, 40),
                    2,
                    cv2.LINE_AA,
                )

                #nilai y
                cv2.putText(
                    frame,
                    f"y:{cy}",
                    (75, 65),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.5,
                    (10, 10, 255),
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    frame,
                    f"id: {ids[0]}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 255),
                    2,
                    cv2.LINE_AA,
                )

                # print(ids, "  ", corners)
            global x,y,z
            x = round(tVec[i][0][0], 1)
            y = round(tVec[i][0][1], 1)
            z = 2.95 * round(tVec[i][0][2], 2)
            titik = cx, cy

            if ids == 5 :
                id = 5

        if marker_IDs == None:
                id = 0

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break

if __name__ == "__main__":
    vehicle = connect("COM15", baud=57600)
    print("connect")

    vehicle.mode = VehicleMode("GUIDED")
    arm_and_takeoff(5)
    print("Berjalan maju awal")
    velocity(0.2, 0, 0, 2)

    t1 = threading.Thread(target=kamera_aruco)
    t2 = threading.Thread(target=cari_aruco)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")
