### Yang sebelumnya Znya salah, gatawu nyapo
### Prbbly need to delete Z but idk
### Kalau mau coba tanpa terbang


from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
from cv2 import aruco
import numpy as np
import threading
def cari_aruco():
    global x, y, z, id, cx, cy, cz5, cz18
    while True:
        global a
        a = 0
#If Aruco jauh dan hanya di 1 sisi
        if id == 5 and cz5 <= 400 and cz5 >= 100:
            print("x : ", cx, "y : ", cy)

            if cx >= 350 and cx <= 640:
                print("ID = 5 Terbaca, bergerak mundur")
              #  velocity(-0.2,0, 0, 1)
            elif cx <= 290 and cx >= 0:
                print("ID = 5 Terbaca, bergerak maju")
               # velocity(0.2, 0, 0, 1)
            elif cy <= 210 and cy >= 0:
                print("ID = 5 Terbaca, bergerak kanan")
             #   velocity(0, 0.2, 0, 1)
            elif cy >= 270 and cy <= 480:
                print("ID = 5 Terbaca, bergerak kiri")
             #   velocity(0, -0.2, 0, 1)

# IF (x,y) ArUco masuk toleransi, tapi masih antara 200-400cm

            elif cx >= 290 and cx <= 350 and cy >= 210 and cy <= 270:
                   print("Masuk toleransi id 5, bergerak turun")
                #  velocity(0, 0, 0.3, 1)

  # ArUco dibawah 100 menggunakan id 18
        if id == 18 and cz18 <= 107 and cz18 >= 50:

            print("x : ", cx, "y : ", cy)

            if cx >= 350 and cx <= 640:
                print("ID = 18 Terbaca, bergerak mundur 18")
                #velocity(-0.1, 0, 0, 1)
            elif cx <= 290 and cx >= 0:
                print("ID = 18 Terbaca, bergerak maju 18")
                #velocity(0.1, 0, 0, 1)
            elif cy <= 210 and cy >= 0:
                print("ID = 18 Terbaca, bergerak kanan 18")
                #velocity(0, 0.1, 0, 1)
            elif cy >= 270 and cy <= 480:
                print("ID = 18 Terbaca, bergerak kiri 18")
                #velocity(0, -0.1, 0, 1)


#######################################  LANDING  ###################################################

        #if id == 5 and cx >= 290 and cx <= 330 and cy >= 210 and cy <= 270 and cz5 <=100 and cz5 >= 50:
         #   print("Diatas ArUco,Landing, A5")
          #  vehicle.mode = VehicleMode("LAND")
          #  vehicle.close()
          #  exit(1)

def kamera_aruco():
    global id, cx, cy, cz5, cz18
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

            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                #kotak kuning ArUco
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
                cz5 = 3 * round(tVec[i][0][2], 2)
                cz18 = 0.6 *round(tVec[i][0][2], 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                #distance
                if ids == 5 :
                    #nilai distance
                    cv2.putText(
                        frame,
                        f"Dist: {(3 * round(tVec[i][0][2], 2))}",
                        (0, 30),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (255, 25, 25),
                        1,
                        cv2.LINE_AA,
                    )

                    #nilai x
                    cv2.putText(
                        frame,
                        f"x:{cx}",
                        (320, 30),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (5, 255, 40),
                        1,
                        cv2.LINE_AA,
                    )

                    #nilai y
                    cv2.putText(
                        frame,
                        f"y:{cy}",
                        (420, 30),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (10, 10, 255),
                        1,
                        cv2.LINE_AA,
                    )
                if ids == 18 :
                    #nlai distacne id 18
                    cv2.putText(
                        frame,
                        f"Dist: { 0.6 * round(tVec[i][0][2], 2)}",
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
                        f"x:{cx}",
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
                        f"y:{cy}",
                        (420, 65),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.5,
                        (10, 10, 255),
                        1,
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
            z = 3 * round(tVec[i][0][2], 2)
            titik = cx, cy

            if ids == 5 :
                id = 5
            if ids == 18:
                id = 18

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break

if __name__ == "__main__":
  #  vehicle = connect("COM15", baud=57600)
  #  print("connect")

  #  vehicle.mode = VehicleMode("GUIDED")
  #  arm_and_takeoff(3)
  #  print("Berjalan maju awal")
  #  velocity(0.2, 0, 0, 2)

    t1 = threading.Thread(target=kamera_aruco)
    t2 = threading.Thread(target=cari_aruco)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("Done!")
