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
    global x, y, z, id
    while True:
        global a
        a = 0
#If Aruconya masih jauh
        if id == 5 and x<=40 and x>=20 and y<=40 and y>=20 :
            print("ID = ", id, "Terbaca, bergerak mundur kiri")
            velocity(-0.2, -0.2, 0, 1)

        elif id == 5 and x>=-40 and x<=-5 and y>=-30 and y<=-5 :
            print("ID = ", id, "Terbaca, bergerak maju kanan")
            velocity(0.2, 0.2, 0, 1)

        elif id == 5 and x <= 40 and x >= 20 and y >= -30 and y <= -5:
            print("ID = ", id, "Terbaca, bergerak mundur kanan")
            velocity(-0.2, 0.2, 0, 1)

        elif id == 5 and x >= -40 and x <= -5 and y <= 40 and y >= 20:
            print("ID = ", id, "Terbaca, bergerak maju kiri")
            velocity(0.2, -0.2, 0, 1)

#If Aruco jauh dan hanya di 1 sisi
        elif id == 5 and x <= 40 and x >= 20 and y <= 6 and y >= 2:
            print("ID = ", id, "Terbaca, bergerak mundur")
            velocity(-0.1,0, 0, 1)

        elif id == 5 and x >= -40 and x <= -5 and y <= 6 and y >= 2:
            print("ID = ", id, "Terbaca, bergerak maju")
            velocity(0.1, 0, 0, 1)

        elif id == 5 and x <= 6 and x >= 2 and y >= -40 and y <= 55:
            print("ID = ", id, "Terbaca, bergerak kanan")
            velocity(0, 0.1, 0, 1)

        elif id == 5 and x <= 6 and x >= 2 and y <= 40 and y >= 20:
            print("ID = ", id, "Terbaca, bergerak kiri")
            velocity(0, -0.1, 0, 1)


#If Aruco terbaca dan sudekat posisi kiri bawah
        elif id == 5 and x <= 15 and x >= -5 and y <= 15 and y >= -5 and z<=400 and z>=300:
            print("ID = ", id, "Terbaca dan dekat, bergerak maju kanan dan turun 500")
            velocity(-0.1, -0.1, 0.1, 1)

        elif id == 5 and x <= 10 and x >= 8 and y <= 10 and y >= 8 and z <=300 and z>=200:
            print("ID = ", id, "Terbaca dan dekat, bergerak maju kanan dan turun 300")
            velocity(-0.1, -0.1, 0.1, 1)

        elif id == 5 and x <= 8 and x >= 1 and y <= 8 and y >= 1 and z <=200 and z>=100:
            print("ID = ", id, "Terbaca dan dekat, bergerak maju kanan dan turun 200")
            velocity(-0.1, -0.1, 0.1, 1)

# If Aruco terbaca dan sudekat posisi kanan atas
        elif id == 5 and x >= -15 and x <= -5 and y >= -15 and y <= -5 and z<=400 and z>=300:
            print("ID = ", id, "Terbaca dan dekat, bergerak mundur kiri dan turun 500")
            velocity(0.1, 0.1, 0.1, 1)

        elif id == 5 and x >= -10 and x <= -3 and y >= -10 and y <= -3 and z <=300 and z>=200:
            print("ID = ", id, "Terbaca dan dekat, bergerak mundur kiri dan turun 300")
            velocity(0.1, 0.1, 0.1, 1)

        elif id == 5 and x >= -8 and x >= -1 and y <= -8 and y >= -1 and z <=200 and z>=100:
            print("ID = ", id, "Terbaca dan dekat, bergerak mundur kiri dan turun 200")
            velocity(0.1, 0.1, 0.1, 1)


# If Aruco terbaca dan sudekat posisi kanan bawah
        elif id == 5 and x <= 15 and x >= -5 and y >= -15 and y <= -5 and z<=400 and z>=300:
            print("ID = ", id, "Terbaca dan dekat, bergerak mundur kanan dan turun 500")
            velocity(-0.1, 0.1, 0.1, 1)

        elif id == 5 and x <= 10 and x >= -3 and y >= -10 and y <= -3 and z <=300 and z>=200:
            print("ID = ", id, "Terbaca dan dekat, bergerak mundur kanan dan turun 300")
            velocity(-0.1, 0.1, 0.1, 1)

        elif id == 5 and x <= 8 and x >= 1 and y >= -8 and y <= -1 and z <=200 and z>=100:
            print("ID = ", id, "Terbaca dan dekat, bergerak mundur kanan dan turun 200")
            velocity(-0.1, 0.1, 0.1, 1)


# If Aruco terbaca dan sudekat posisi kiri atas
        elif id == 5 and x >= -15 and x <= -5 and y <= 15 and y >= -5 and z<=400 and z>=300:
            print("ID = ", id, "Terbaca dan dekat, bergerak maju kiri dan turun 500")
            velocity(0.1, -0.1, 0.1, 1)

        elif id == 5 and x >= -10 and x <= -3 and y <= 10 and y >= -3 and z <=300 and z>=200:
            print("ID = ", id, "Terbaca dan dekat, bergerak maju kiri dan turun 300")
            velocity(0.1, -0.1, 0.1, 1)

        elif id == 5 and x >= -8 and x <= -1 and y <= 8 and y >= 1 and z <=200 and z>=100:
            print("ID = ", id, "Terbaca dan dekat, bergerak maju kiri dan turun 200")
            velocity(0.1, -0.1, 0.1, 1)

# IF (x,y) ArUco masuk toleransi, tapi masih diatas 100 cm
        elif id == 5 and x <= 6 and x >= 2 and y <= 6 and y >= 2 and z <= 300 and z >= 100:
            print("ID = ", id, "Diatas, jarak diatas 100 cm, bergerak turun")
            velocity(0, 0, 0.2, 1)


############################### IF ARUCO SUDAH TURUN JADI 100 CM ###############################################


# If Aruco 1m, kurang ke depan
        elif id == 5 and x >= -10 and x <= 2 and y <= 6 and y >= 2 and z <=100 and z>=50:
            print("ID = ", id, "Diatas, jarak 100 cm, bergerak maju")
            velocity(0.1, 0, 0, 1)
# If Aruco 1m, kurang ke belakang
        elif id == 5 and x <= 10 and x >= 2 and y <= 6 and y >= 2 and z <=100 and z>=50:
            print("ID = ", id, "Diatas, jarak 100 cm, bergerak mundur")
            velocity(-0.1, 0, 0, 1)
# If Aruco 1m, kurang ke kanan
        elif id == 5 and x <= 6 and x >= 2 and y <= 10 and y >= 2 and z <=100 and z>=50:
            print("ID = ", id, "Diatas, jarak 100 cm, bergerak kanan")
            velocity(0, 0.1, 0, 1)
# If Aruco 1m, kurang ke kiri
        elif id == 5 and x <= 6 and x >= 2 and y >= -10 and y <= -2 and z <=100 and z>=50:
            print("ID = ", id, "Diatas, jarak 100 cm, bergerak kiri")
            velocity(0, -0.1, 0, 1)


#######################################  LANDING  ###################################################

        elif id == 5 and x <= 6 and x >= 2 and y <= 6 and y >= 2 and z <=100 and z>=50:
            print("Diatas ArUco,Landing")
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
            exit(1)

        for a in range(50):
            if id == 0:
                print(id)
                print("Berjalan Maju")
                velocity(0.2, 0, 0, 1)
                a += 1
                print(a)
                while a == 50:
                    time.sleep(1)
                    print("ArUco Tidak ditemukan, Landing")
                    vehicle.mode = VehicleMode("LAND")
                    vehicle.close()
                    exit(1)

def kamera_aruco():
    global id
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
        global id, z
        ret, frame = cap.read()
        if not ret:
            break
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )

        if marker_corners:
            global rVec, tVec, ids
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
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

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
                    f"x:{round(tVec[i][0][0], 1)}",
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
                    f"y:{round(tVec[i][0][1], 1)}",
                    (75, 65),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.5,
                    (10, 10, 255),
                    2,
                    cv2.LINE_AA,
                )

                # ID Pada Marker
                poir = cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
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
    arm_and_takeoff(3)

    t1 = threading.Thread(target=kamera_aruco)
    t2 = threading.Thread(target=cari_aruco)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")
