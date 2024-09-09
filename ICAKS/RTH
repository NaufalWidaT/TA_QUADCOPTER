import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
from cv2 import aruco
import numpy as np
import threading

home_location = None


def arm_and_takeoff(aTargetAltitude):
    global home_location

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

    # Save home location
    home_location = vehicle.location.global_frame
    print("Home location saved: ", home_location)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def velocityd(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000111,  # type_mask
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # m/s
        0, 0, 0,  # x, y, z acceleration
        0, 0)
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def return_to_home():
    if home_location is not None:
        print("Returning to Home location...")
        vehicle.mode = VehicleMode("RTL")
    else:
        print("Home location is not set!")

vehicle = connect("COM15", baud=57600)
print("Connected")

# QC mode guided
vehicle.mode = VehicleMode("GUIDED")

# Take off
arm_and_takeoff(2)
time.sleep(1)

print("Maju Awal")
velocityd(0.3, 0, 0, 10)
time.sleep(1)
print("Bergerak nganan")
velocityd(0,-0.3,0,8)
time.sleep(1)

# Return to home after completing the mission
return_to_home()

print("Turun setelah RTH")
velocityd(0,0,0.3,4)
time.sleep(1)

vehicle.mode = VehicleMode("LAND")
print("Landing")
vehicle.close()
exit(1)

cv2.destroyAllWindows()
