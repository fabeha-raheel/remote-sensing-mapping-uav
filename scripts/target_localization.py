#!/usr/bin/env python

import math
import pickle
import rospy
import rospkg
import sys

from remote_sensing_mapping_uav.msg import Target

try:
    pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')
except:
    print("Unable to locate package path. Try sourcing your ROS workspace.")
    sys.exit()

LOG_FILEPATH = pkg_path + r'/logs/data.pickle'

TARGETS = []

camera_hfov = 60
camera_w = 640
camera_h = 480
camera_vfov = round(camera_hfov * (camera_w/camera_h), 2)

gw = (2*math.tan(math.radians(camera_hfov/2))) / camera_w
gh = (2*math.tan(math.radians(camera_vfov/2))) / camera_h

R = 6378137.0

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two GPS locations.
    
    Locations should be passed as a tuple in the form (lat, long).

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2[0] - aLocation1[0]
    dlong = aLocation2[1] - aLocation1[1]
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def localization_callback(mssg):
    GSD_x = mssg.altitude * gw
    GSD_y = mssg.altitude * gh

    cx, cy, w, h = mssg.x_center, mssg.y_center, mssg.width, mssg.height
    psi = math.radians(mssg.heading)
    dx = camera_w - cx
    dy = camera_h - cy

    P_c_x = GSD_x*dx
    P_c_y = GSD_y*dy

    P_N = math.cos(psi)*P_c_x - math.sin(psi)*P_c_y
    P_E = math.sin(psi)*P_c_x + math.cos(psi)*P_c_y

    delta_lat = P_N / R
    delta_long = P_E / (math.cos(math.radians(mssg.latitude)))

    target_lat = mssg.latitude + delta_lat
    target_long = mssg.longitude + delta_long
    target_dist = mssg.distance_from_image_center
    target_class = mssg.class_name

    location = (target_lat, target_long)

    target_present = False

    for target in TARGETS:

        gps_distance = get_distance_metres(location, target[1])
        print(gps_distance)
        
        if gps_distance < 2:
            target_present = True

            if target_dist < target[0]:
                #update the location
                # print("Updating landmine")
                index = TARGETS.index(target)
                TARGETS[index] = (target_dist,location,target_class)
                write_to_log(TARGETS)
                # print("Updated existing landmine...")
                # print(drone_data.landmines)
                break
            else:
                #no need to update and check with other landmine
                break
        
    if target_present == False:
        detection = (target_dist, location, target_class)
        TARGETS.append(detection)
        write_to_log(TARGETS)
        # print("New land mine found...")
        # print(drone_data.landmines)

    print(TARGETS)
    print()

def write_to_log(data):

    f = open(LOG_FILEPATH, 'wb')
    pickle.dump(data, f)
    f.close()


if __name__ == "__main__":

    write_to_log(TARGETS)

    rospy.init_node('Target_Geolocation_Estimation_Node') 

    Target_Subscriber = rospy.Subscriber('target_detection/target', Target, localization_callback)

    rospy.spin()
