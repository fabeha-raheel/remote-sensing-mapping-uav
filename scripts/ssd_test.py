#!/usr/bin/env python

import cv2
import rospkg

pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')

thres = 0.6 # Threshold to detect object
nms_thres = 0.6

classNames= []
classFile = pkg_path + '/include/coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
# print(len(classNames))
# print(classNames)
configPath = pkg_path + '/include/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = pkg_path + '/include/frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


def getobjects(img, draw=True, objects = []):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold = nms_thres)
    # print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo =[]
    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId-1]
            if className in objects:
                objectInfo.append([box, className]) 
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,className.upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
    return img, objectInfo

if __name__ == "__main__":


    img = cv2.imread('/home/fabeha/Pictures/1.png')
    # cap = cv2.VideoCapture(pkg_path + '/output_video.avi')
    # cap.set(3,640)
    # cap.set(4,480)
    # cap.set
    # while True:
    # success,img = cap.read()
    result,objectInfo = getobjects(img)
    NumberofHuman = len(objectInfo)
    print(objectInfo)
    cv2.imshow("Output",img)
    cv2.waitKey(0)
