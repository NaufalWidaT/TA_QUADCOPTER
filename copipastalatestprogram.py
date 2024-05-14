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
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(5)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an abs
        # olute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        30,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111,  # type_mask
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # m/s
        0, 0, 0,  # x, y, z acceleration
        0, 0)
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def cari_aruco():
    global x, y, z, id, cx5, cx18, cy5, cy18, cz5, cz18
    while True:
        global a
        a = 0
        # ArUco setelah takeoff (diatas 200cm)
        if vehicle.location.global_relative_frame.alt > 2.2:

            if id == 5 and cz5 <= 450 and cz5 >= 200:
                print("x : ", cx5, "y : ", cy5, "tinggi: ", vehicle.location.global_relative_frame.alt)

                if cx5 >= 350 and cx5 <= 640:
                    print("ID = 5 Terbaca, bergerak mundur")
                    velocity(-0.2, 0, 0, 1)
                elif cx5 <= 290 and cx5 >= 0:
                    print("ID = 5 Terbaca, bergerak maju")
                    velocity(0.2, 0, 0, 1)
                elif cy5 <= 210 and cy5 >= 0:
                    print("ID = 5 Terbaca, bergerak kanan")
                    velocity(0, 0.2, 0, 1)
                elif cy5 >= 270 and cy5 <= 480:
                    print("ID = 5 Terbaca, bergerak kiri")
                    velocity(0, -0.2, 0, 1)

                # IF (x,y) ArUco masuk toleransi, tapi masih antara 200-400cm

                elif cx5 >= 290 and cx5 <= 350 and cy5 >= 210 and cy5 <= 270:
                    print("Masuk toleransi, bergerak turun")
                    velocity(0, 0, 0.3, 1)

        # ArUco dibawah 100 menggunakan id 18
        if vehicle.location.global_relative_frame.alt < 2.2:

            if id == 18 and cz18 <= 200 and cz18 >= 50:

                print("x : ", cx18, "y : ", cy18, "tinggi: ", vehicle.location.global_relative_frame.alt)

                if cx18 >= 160 and cx18 <= 640:
                    print("ID = 18 Terbaca, bergerak mundur 18")
                    velocity(-0.1, 0, 0, 1)
                elif cx18 <= 130 and cx18 >= 0:
                    print("ID = 18 Terbaca, bergerak maju 18")
                    velocity(0.1, 0, 0, 1)
                elif cy18 <= 210 and cy18 >= 0:
                    print("ID = 18 Terbaca, bergerak kanan 18")
                    velocity(0, 0.1, 0, 1)
                elif cy18 >= 270 and cy18 <= 480:
                    print("ID = 18 Terbaca, bergerak kiri 18")
                    velocity(0, -0.1, 0, 1)

                elif cx18 >= 130 and cx18 <= 160 and cy18 >= 210 and cy18 <= 270:
                    print("Masuk toleransi, bergerak turun")
                    velocity(0, 0, 0.2, 1)

            if id == 18 and cz18 <= 50 :

                print("x : ", cx18, "y : ", cy18, "tinggi: ", vehicle.location.global_relative_frame.alt)

                if cx18 >= 160 and cx18 <= 640:
                    print("ID = 18 Terbaca, bergerak mundur 18")
                    velocity(-0.1, 0, 0, 1)
                elif cx18 <= 130 and cx18 >= 0:
                    print("ID = 18 Terbaca, bergerak maju 18")
                    velocity(0.1, 0, 0, 1)
                elif cy18 <= 210 and cy18 >= 0:
                    print("ID = 18 Terbaca, bergerak kanan 18")
                    velocity(0, 0.1, 0, 1)
                elif cy18 >= 270 and cy18 <= 480:
                    print("ID = 18 Terbaca, bergerak kiri 18")
                    velocity(0, -0.1, 0, 1)

                elif cx18 >= 130 and cx18 <= 160 and cy18 >= 210 and cy18 <= 270:
                    print("Masuk toleransi, bergerak turun")
                    velocity(0, 0, 0.3, 1)

        #######################################  LANDING  ###################################################

        #if id == 18 and cx18 >= 130 and cx18 <= 160 and cy18 >= 210 and cy18 <= 270 and cz18 <= 100 and cz18 >= 50:
        #    print("Diatas ArUco,Landing, A5")
        #    vehicle.mode = VehicleMode("LAND")
        #    vehicle.close()
        #    exit(1)


def kamera_aruco():
    global id, cx5, cx18, cy5, cy18, cz5, cz18
    calib_data_path = "MultiMatrix.npz"
    calib_data = np.load(calib_data_path)
    print(calib_data.files)

    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]

    MARKER_SIZE = 8  # CM
    marker_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    param_markers = cv2.aruco.DetectorParameters()
    cap = cv2.VideoCapture(0)

    while True:
        global id

        ret, frame = cap.read()
        if not ret:
            break
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )

        if marker_corners:
            global rVec, tVec, ids, cx, cy, cz5, cz18
            rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)

            for ids, corners5, i in zip(marker_IDs, marker_corners, total_markers):
                # kotak kuning ArUco5
                # distance
                if ids == 5:
                    cv2.polylines(
                        frame, [corners5.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                    )
                    corners5 = corners5.reshape(4, 2)
                    corners5 = corners5.astype(int)
                    top_right5 = corners5[1].ravel()
                    top_left5 = corners5[0].ravel()
                    bottom_right5 = corners5[2].ravel()
                    bottom_left5 = corners5[3].ravel()

                    # Nambah titik tengah di ArUco Marker
                    cx5 = int((top_left5[0] + bottom_right5[0]) / 2)
                    cy5 = int((top_left5[1] + bottom_right5[1]) / 2)
                    cz5 = 3 * round(tVec[i][0][2], 2)
                    cv2.circle(frame, (cx5, cy5), 5, (0, 0, 255), -1)

                    # nilai distance
                    cv2.putText(
                        frame,
                        f"Dist 5: {(3 * round(tVec[i][0][2], 2))}",
                        (0, 30),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (255, 25, 25),
                        1,
                        cv2.LINE_AA,
                    )

                    # nilai x
                    cv2.putText(
                        frame,
                        f"x:{cx5}",
                        (320, 30),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (5, 255, 40),
                        1,
                        cv2.LINE_AA,
                    )

                    # nilai y
                    cv2.putText(
                        frame,
                        f"y:{cy5}",
                        (420, 30),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (10, 10, 255),
                        1,
                        cv2.LINE_AA,
                    )

                    # id
                    cv2.putText(
                        frame,
                        f"id: {ids[0]}",
                        top_right5,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (200, 100, 255),
                        2,
                        cv2.LINE_AA,
                    )

            for ids, corners18, i in zip(marker_IDs, marker_corners, total_markers):
                if ids == 18:
                    # kotak ijo ArUco 18
                    cv2.polylines(
                        frame, [corners18.astype(np.int32)], True, (0, 255, 0), 4, cv2.LINE_AA
                    )
                    corners18 = corners18.reshape(4, 2)
                    corners18 = corners18.astype(int)
                    top_right18 = corners18[1].ravel()
                    top_left18 = corners18[0].ravel()
                    bottom_right18 = corners18[2].ravel()
                    bottom_left18 = corners18[3].ravel()

                    # Nambah titik tengah di ArUco Marker
                    cx18 = int((top_left18[0] + bottom_right18[0]) / 2)
                    cy18 = int((top_left18[1] + bottom_right18[1]) / 2)
                    cz18 = 0.6 * round(tVec[i][0][2], 2)
                    cv2.circle(frame, (cx18, cy18), 5, (0, 0, 255), -1)

                    # nlai distacne id 18
                    cv2.putText(
                        frame,
                        f"Dist 18: {0.6 * round(tVec[i][0][2], 2)}",
                        (0, 65),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (255, 25, 25),
                        1,
                        cv2.LINE_AA,
                    )

                    # nilai x 18
                    cv2.putText(
                        frame,
                        f"x:{cx18}",
                        (320, 65),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (5, 255, 40),
                        1,
                        cv2.LINE_AA,
                    )

                    # nilai y 18
                    cv2.putText(
                        frame,
                        f"y:{cy18}",
                        (420, 65),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (10, 10, 255),
                        1,
                        cv2.LINE_AA,
                    )
                    # id
                    cv2.putText(
                        frame,
                        f"id: {ids[0]}",
                        top_right18,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (200, 100, 255),
                        2,
                        cv2.LINE_AA,
                    )

                # print(ids, "  ", corners)
            global x, y, z
            x = round(tVec[i][0][0], 1)
            y = round(tVec[i][0][1], 1)
            z = 3 * round(tVec[i][0][2], 2)

            if ids == 5:
                id = 5
            if ids == 18:
                id = 18

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break


if __name__ == "__main__":
    vehicle = connect("COM15", baud=57600)
    print("connect")

    vehicle.mode = VehicleMode("GUIDED")
    arm_and_takeoff(4)
    print("Berjalan maju awal")
    velocity(0.2, 0, 0, 2)

    t1 = threading.Thread(target=kamera_aruco)
    t2 = threading.Thread(target=cari_aruco)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")
