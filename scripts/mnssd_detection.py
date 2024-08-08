#!/usr/bin/env python

import cv2
import numpy as np
# import imutils
import math
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Image
from nav_msgs.msg import Odometry
from remote_sensing_mapping_uav.msg import Target

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import pickle
import rospkg
import sys

import drone_data

TARGETS_LIST = ['person', 'car', 'bus', 'truck', 'mouse', 'cellphone']

try:
    pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')
except:
    print("Unable to locate package path. Try sourcing your ROS workspace.")
    sys.exit()

thres = 0.6 # Threshold to detect object
nms_thres = 0.6

classNames= []
classFile = pkg_path + '/include/coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
# print(len(classNames))
configPath = pkg_path + '/include/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = pkg_path + '/include/frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

bridge = CvBridge()

def GPS_Subscriber_callback(mssg):

    drone_data.latitude = mssg.latitude
    drone_data.longitude = mssg.longitude

def Local_Position_Subscriber_callback(mssg):
    drone_data.altitude = mssg.pose.pose.position.z

def HDG_subscriber_callback(mssg):
    drone_data.heading = mssg.data

def roscamera_callback(mssg):
    try:
        drone_data.roscamera_cvImage = bridge.imgmsg_to_cv2(mssg, "bgr8")
    except CvBridgeError as e:
        print('Error converting ROS Image to CV: ', e)

def target_detection(frame, frame_center, publisher):

    if len(TARGETS_LIST) == 0:
        print("No targets specified")
        return frame

    classIds, confs, bbox = net.detect(frame,confThreshold=thres,nmsThreshold = nms_thres)
    
    result = frame.copy()

    if len(classIds) != 0:
        location = (drone_data.latitude, drone_data.longitude, drone_data.altitude, drone_data.heading)
        largest_target_area = 0
        targetInfo = None

        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            
            className = classNames[classId-1]
            
            if className in TARGETS_LIST:
                # save the target with largest area in the frame
                area = box[2]*box[3]
                if area > largest_target_area:
                    targetInfo = [box, className, confidence, location] 
                    largest_target_area = area

        if targetInfo is None:
            return frame
        else:
            # Publish the target information
            x,y,w,h = targetInfo[0][0], targetInfo[0][1], targetInfo[0][2], targetInfo[0][3]
            cx, cy = int(x + (w//2)), int(y + (h//2))
            target_msg = Target()
            target_msg.x_center = int(cx)
            target_msg.y_center = int(cy)
            target_msg.width = int(w)
            target_msg.height = int(h)
            target_msg.class_name = targetInfo[1]
            target_msg.confidence = round(targetInfo[2], 2)
            target_msg.latitude = targetInfo[3][0]
            target_msg.longitude = targetInfo[3][1]
            target_msg.altitude = round(targetInfo[3][2], 2)
            target_msg.heading = round(targetInfo[3][3], 2)
            target_msg.distance_from_image_center = int(math.sqrt((frame_center[1]-cx)**2 + (frame_center[0]-cy)**2))

            publisher.publish(target_msg)

            # Draw and output frame
            cv2.rectangle(result,targetInfo[0],color=(0,255,0),thickness=2)
            cv2.putText(result,targetInfo[1].upper(),(targetInfo[0][0]+10,targetInfo[0][1]+30),
            cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
            cv2.putText(result,str(round(targetInfo[2]*100,2)),(targetInfo[0][0]+200,targetInfo[0][1]+30),
            cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
            return result

    else:
        return frame

if __name__ == "__main__":

    drone_data.targets = []

    # write_to_log(drone_data.targets)

    rospy.init_node('Target_Detection_Node') 

    GPS_Subscriber=rospy.Subscriber('/mavros/global_position/global',NavSatFix, GPS_Subscriber_callback)
    Local_Position_Subscriber = rospy.Subscriber('/mavros/global_position/local',Odometry, Local_Position_Subscriber_callback)
    Compass_Hdg_Subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, HDG_subscriber_callback)
    roscamera=rospy.Subscriber("/webcam/image_raw", Image, roscamera_callback)

    Target_Publisher = rospy.Publisher('target_detection/target', Target, queue_size=10)

    # cap = cv2.VideoCapture(pkg_path + '/output_video.avi')
    # cap.set(3,640)
    # cap.set(4,480)

    # success,img = cap.read()
    # drone_data.roscamera_cvImage = img
    
    while drone_data.roscamera_cvImage is None:
        # wait
        pass

    FRAME_WIDTH, FRAME_HEIGHT, _ = drone_data.roscamera_cvImage.shape
    print(FRAME_WIDTH)
    print(FRAME_HEIGHT)
    frame_center = (int(FRAME_WIDTH/2) , int(FRAME_HEIGHT/2))
    print("Stored frame data....")

    scale_percent = 80 # percent of original size
    width = 500
    height = 500
    dim = (width, height)

    success = True

    while not rospy.is_shutdown():

        # success,img = cap.read()
        
        if success:
            # Target Detection Codeblock
            detection = target_detection(drone_data.roscamera_cvImage, frame_center, Target_Publisher)
            # detection = target_detection(img, frame_center, Target_Publisher)

            # Display the resulting frame
            # resized = cv2.resize(detection, dim, interpolation = cv2.INTER_AREA)
            cv2.imshow('Drone Video Feed', detection)
            if cv2.waitKey(1) == ord('q'):
                break

    # When everything done, release the capture
    cv2.destroyAllWindows()