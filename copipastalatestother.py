from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
from cv2 import aruco
import numpy as np
import threading


def cari_aruco():
    global x, y, z, id, cx5, cx18, cy5, cy18, cz5, cz18, ukuran5, ukuran18
    while True:
        global a
        a = 0
        if ukuran5 >= ukuran18:
# ArUco setelah takeoff (diatas 200cm)
            if id == 5 :
                print("x : ", cx5, "y : ", cy5)

                if cx5 >= 350 and cx5 <= 640:
                    print("ID = 5 Terbaca, bergerak mundur")
               #     velocity(-0.2, 0, 0, 1)
                elif cx5 <= 290 and cx5 >= 0:
                    print("ID = 5 Terbaca, bergerak maju")
               #     velocity(0.2, 0, 0, 1)
                elif cy5 <= 210 and cy5 >= 0:
                    print("ID = 5 Terbaca, bergerak kanan")
               #     velocity(0, 0.2, 0, 1)
                elif cy5 >= 270 and cy5 <= 480:
                    print("ID = 5 Terbaca, bergerak kiri")
               #     velocity(0, -0.2, 0, 1)

                # IF (x,y) ArUco masuk toleransi, tapi masih antara 200-400cm

                elif cx5 >= 290 and cx5 <= 350 and cy5 >= 210 and cy5 <= 270:
                    print("Masuk toleransi, bergerak turun")
               #     velocity(0, 0, 0.3, 1)

    # ArUco dibawah 100 menggunakan id 18

        else :
            if id == 18 :

                print("x : ", cx18, "y : ", cy18)

                if cx18 >= 160 and cx18 <= 640:
                    print("ID = 18 Terbaca, bergerak mundur 18")
                #    velocity(-0.1, 0, 0, 1)
                elif cx18 <= 130 and cx18 >= 0:
                    print("ID = 18 Terbaca, bergerak maju 18")
                #    velocity(0.1, 0, 0, 1)
                elif cy18 <= 210 and cy18 >= 0:
                    print("ID = 18 Terbaca, bergerak kanan 18")
                #    velocity(0, 0.1, 0, 1)
                elif cy18 >= 270 and cy18 <= 480:
                    print("ID = 18 Terbaca, bergerak kiri 18")
                #    velocity(0, -0.1, 0, 1)

                elif cx18 >= 130 and cx18 <= 160 and cy18 >= 210 and cy18 <= 270:
                    print("Masuk toleransi, bergerak turun")
                #    velocity(0, 0, 0.2, 1)
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

    MARKER_SIZE5 = 40 # CM
    MARKER_SIZE18 = 10
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
            global rVec, tVec, ids, cz5, cz18
           # rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
           #     marker_corners, MARKER_SIZE, cam_mat, dist_coef)
            total_markers = range(0, marker_IDs.size)

            for ids, corners5, i in zip(marker_IDs, marker_corners, total_markers):
                # kotak kuning ArUco5
                # distance
                if ids == 5:
                    rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        marker_corners, MARKER_SIZE5, cam_mat, dist_coef
                    )
                    cv2.polylines(
                        frame, [corners5.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                    )
                    corners5 = corners5.reshape(4, 2)
                    corners5 = corners5.astype(int)

                    top_right5 = corners5[1].ravel()
                    top_left5 = corners5[0].ravel()
                    bottom_right5 = corners5[2].ravel()
                    bottom_left5 = corners5[3].ravel()

                    ukuran5 = ((top_left5[0] + top_right5[0] + bottom_left5[0] + bottom_right5[0])
                               + (top_left5[1] + top_right5[1] + bottom_left5[1] + bottom_right5[1]))
                    #ukbg = cv2.arcLength(ukuran5, True)

                    # Nambah titik tengah di ArUco Marker
                    cx5 = int((top_left5[0] + bottom_right5[0]) / 2)
                    cy5 = int((top_left5[1] + bottom_right5[1]) / 2)
                    cz5 = 3 * round(tVec[i][0][2], 2)
                    cv2.circle(frame, (cx5, cy5), 5, (0, 0, 255), -1)

                    # nilai distance
                    cv2.putText(
                        frame,
                        f"Dist 5: {0.55 * (round(tVec[i][0][2], 2))}",
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
                        f"Size = {ukuran5}",
                        top_right5,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )



            for ids, corners18, i in zip(marker_IDs, marker_corners, total_markers):
                if ids == 18:
                    rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        marker_corners, MARKER_SIZE18, cam_mat, dist_coef
                    )

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
                    ukuran18 = ((top_left18[0] + top_right18[0] + bottom_left18[0] + bottom_right18[0])
                                + (top_left18[1] + top_right18[1] + bottom_left18[1] + bottom_right18[1])/9)


                    # Nambah titik tengah di ArUco Marker
                    cx18 = int((top_left18[0] + bottom_right18[0]) / 2)
                    cy18 = int((top_left18[1] + bottom_right18[1]) / 2)
                    cz18 = 0.6 * round(tVec[i][0][2], 2)
                    cv2.circle(frame, (cx18, cy18), 5, (0, 0, 255), -1)

                    # nlai distacne id 18
                    cv2.putText(
                        frame,
                        f"Dist 18: {0.5 * round(tVec[i][0][2], 2)}",
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
                    #cv2.putText(
                    #   frame,
                    #    f"id: {ids[0]}",
                    #    top_right18,
                    #    cv2.FONT_HERSHEY_PLAIN,
                    #    1.3,
                    #    (200, 100, 255),
                    #   2,
                    #    cv2.LINE_AA,
                    #)

                    cv2.putText(
                        frame,
                        f"Size = {ukuran18}",
                        top_left18,
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (255, 255, 255),
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
    #vehicle = connect("COM15", baud=57600)
    #print("connect")

    #vehicle.mode = VehicleMode("GUIDED")
    #arm_and_takeoff(4)
    #print("Berjalan maju awal")
   # velocity(0.2, 0, 0, 2)

    t1 = threading.Thread(target=kamera_aruco)
    t2 = threading.Thread(target=cari_aruco)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")
