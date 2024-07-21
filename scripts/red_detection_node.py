#!/usr/bin/env python

import cv2
import numpy as np
import imutils
import math
import rospy
from sensor_msgs.msg import NavSatFix, Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import pickle

import drone_data


DETECT_THRESHOLD = 20
LOG_FILEPATH = r'/home/ugv/rtab_ws/src/remote-sensing-mapping-uav/logs/data.pickle'

bridge = CvBridge()

def GPS_Subscriber_callback(mssg):

    drone_data.latitude = mssg.latitude
    drone_data.longitude = mssg.longitude
    drone_data.altitude = mssg.altitude

def roscamera_callback(mssg):
    try:
        drone_data.roscamera_cvImage = bridge.imgmsg_to_cv2(mssg, "bgr8")
    except CvBridgeError as e:
        print('Error converting ROS Image to CV: ', e)

def landmine_detection(frame, frame_center):

    distance = 0

    # lower boundary RED color range values; Hue (0 - 10)
    HSV_LOWER_1 = np.array([0, 100, 20])
    HSV_UPPER_1 = np.array([10, 255, 255])
    
    # upper boundary RED color range values; Hue (160 - 180)
    HSV_LOWER_2 = np.array([160,100,20])
    HSV_UPPER_2 = np.array([179,255,255])

    result = frame.copy()
    
    result = cv2.GaussianBlur(result, (13,13), 0)
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_mask = cv2.inRange(image, HSV_LOWER_1, HSV_UPPER_1)
    upper_mask = cv2.inRange(image, HSV_LOWER_2, HSV_UPPER_2)
    
    full_mask = lower_mask + upper_mask
    full_mask = cv2.erode(full_mask, None, iterations=2)
    full_mask = cv2.dilate(full_mask, None, iterations=2)

    result = cv2.bitwise_and(result, result, mask=full_mask)

    contours = cv2.findContours(full_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(max_contour)
        bbox = cv2.boundingRect(max_contour)

        moment = cv2.moments(max_contour)
        center = (int(moment["m10"] / moment["m00"]), int(moment["m01"] / moment["m00"]))

        if radius > DETECT_THRESHOLD:

            # Computing Distance from center
            distance = int(math.sqrt((frame_center[1]-center[1])**2 + (frame_center[0]-center[0])**2))

            # Save detection
            location = (drone_data.latitude, drone_data.longitude)

            landmine_present = False

            for landmine in drone_data.landmines:

                gps_distance = get_distance_metres(location, landmine[1])
                
                if gps_distance < 2:
                    landmine_present = True

                    if distance < landmine[0]:
                        #update the location
                        # print("Updating landmine")
                        index = drone_data.landmines.index(landmine)
                        drone_data.landmines[index] = (distance,location)
                        write_to_log(drone_data.landmines)
                        # print("Updated existing landmine...")
                        # print(drone_data.landmines)
                        break
                    else:
                        #no need to update and check with other landmine
                        break
                
            if landmine_present == False:
                detection = (distance, location)
                drone_data.landmines.append(detection)
                write_to_log(drone_data.landmines)
                # print("New land mine found...")
                # print(drone_data.landmines)

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 2)
            cv2.circle(frame, center, 5, (255,0,0), -1)
            cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[0]+bbox[2]), int(bbox[1]+bbox[3])), (0,0,255), 1)
            cv2.line(frame, frame_center, center, (0,255,0), 2) 
    
    return frame, distance

def write_to_log(data):

    f = open(LOG_FILEPATH, 'wb')
    pickle.dump(data, f)
    f.close()
        
    # with open('/home/ugv/lmd_ws/src/lmd_sim/logs/lmd_data.pickle', 'wb') as f:
    #     print("Writing data to log: ", data)
    #     pickle.dump(data, f)

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


if __name__ == "__main__":

    drone_data.landmines = []

    write_to_log(drone_data.landmines)

    rospy.init_node('Landmine_Detection') 

    GPS_Subscriber=rospy.Subscriber('/mavros/global_position/global',NavSatFix, GPS_Subscriber_callback)
    roscamera=rospy.Subscriber("/webcam/image_raw", Image, roscamera_callback)
    
    while drone_data.roscamera_cvImage is None:
        # wait
        pass

    FRAME_WIDTH, FRAME_HEIGHT, _ = drone_data.roscamera_cvImage.shape
    frame_center = (int(FRAME_WIDTH/2) , int(FRAME_HEIGHT/2))
    print("Stored frame data....")

    scale_percent = 80 # percent of original size
    width = 500
    height = 500
    dim = (width, height)

    while True:
        
        # Landmine Detection Codeblock
        detection, distance = landmine_detection(drone_data.roscamera_cvImage, frame_center)

        # Display the resulting frame
        # resize image
        resized = cv2.resize(detection, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow('Drone Video Feed', resized)
        if cv2.waitKey(1) == ord('q'):
            break

    # When everything done, release the capture
    cv2.destroyAllWindows()