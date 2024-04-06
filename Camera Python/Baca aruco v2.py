import time

import cv2
from cv2 import aruco
import numpy as np
import math

calib_data_path = "MultiMatrix.npz"
calib_data = np.load(calib_data_path)
print(calib_data.files)

font = cv2.FONT_HERSHEY_PLAIN

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 8 #CM
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
        rVec, tVec, _= cv2.aruco.estimatePoseSingleMarkers(
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

            cv2.putText(
                frame,
                f"Dist: {2.95 * round(tVec[i][0][2], 2)}",
                (0,30),
                font,
                1.5,
                (255, 25, 25),
                2,
                cv2.LINE_AA,
            )
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

            #Draw the pose of the marker
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
        x = round(tVec[i][0][0], 1)
        y = round(tVec[i][0][1], 1)
        z = 2.95 * round(tVec[i][0][2], 2)

        if ids == 5 :
            print("x = ", x)
            print("y = ", y)
            if x <= 5 and x >= 0 and y <= 5 and y >= 0 :
                print("Ini seharusnya ditengah")
            elif x <= 20 and x >= 5 and y <= 20 and y >= 5 :
                print("Ini seharusnya dikanan atas")
            elif x >= -20 and x <= 5 and y >= -20 and y <= 5 :
                print("Ini seharusnya dikiri bawah")
            elif x >= -20 and x <= 5 and y <= 20 and y >= 5 :
                print("Ini seharusnya dikanan bawah")
            elif x <= 20 and x >= 5 and y >= -20 and y <= 5 :
                print("Ini seharusnya dikiri atas")


        if ids == 21 and z >= 10 and z <= 20:
            print("Antara 10-20 cm")
        elif ids == 21 and z >= 20 and z <= 30:
            print("Antara 20-30 cm")
        elif ids == 21 and z >= 30 and z <= 40:
            print("Antara 30-40 cm")
        elif ids == 21 and z >= 40 and z <= 50:
            print("Antara 40-50 cm")
        elif ids == 21 and z >= 50 and z <= 60:
            print("Antara 50-60 cm")
        elif ids == 21 and z >= 60 and z <= 70:
            print("Antara 60-70 cm")
        elif ids == 21 and z >= 70 and z <= 80:
            print("Antara 70-80 cm")
        elif ids == 21 and z >= 100 and z <= 150:
            print("Antara 100-150 cm")
        elif ids == 21 and z >= 150 and z <= 200:
            print("Antara 150-200 cm")
        elif ids == 21 and z >= 200 and z <= 250:
            print("Antara 200-250 cm")
        elif ids == 21 and z >= 250 and z <= 300:
            print("Antara 250-300 cm")
        elif ids == 21 and z >= 300 and z <= 350:
            print("Antara 300-350 cm")
        elif ids == 21 and z >= 350 and z <= 400:
            print("Antara 350-400 cm")
        else:
            print("Luwih seko 400")

    cv2.line(frame, (0, 240), (640, 240), (0, 0, 255), 2)
    cv2.line(frame, (0, 120), (640, 120), (0, 0, 255), 2)
    cv2.line(frame, (0, 360), (640, 360), (0, 0, 255), 2)
    cv2.line(frame, (320, 0), (320, 480), (0, 0, 255), 2)
    cv2.line(frame, (160, 0), (160, 480), (0, 0, 255), 2)
    cv2.line(frame, (480, 0), (480, 480), (0, 0, 255), 2)





    cv2.imshow("frame", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        break




#def distance():
 #   global z, id, dist_coef

    #def yaw():
    #    print("menambah yaw hingga detect")
   # if marker_IDs == None:
     #   yaw()
     #   print("none")

           #(id != 21) and (id != 7) and (id != 20) and (id != 1)



cap.release()
cv2.destroyAllWindows()
