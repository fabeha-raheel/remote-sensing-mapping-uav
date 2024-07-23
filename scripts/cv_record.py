#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRecorder:
    def __init__(self):
        rospy.init_node('image_recorder', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.image_callback)
        self.record_video = True  # Set to False if you want to save as individual images
        self.video_writer = None
        self.frame_rate = 30  # Adjust as needed

        if self.record_video:
            self.init_video_writer()

    def init_video_writer(self):
        # Define the codec and create VideoWriter object
        # Adjust parameters as needed (e.g., filename, codec, frame size)
        filename = 'output_video.avi'
        codec = cv2.VideoWriter_fourcc(*'XVID')
        frame_size = (640, 480)  # Adjust according to your image size
        self.video_writer = cv2.VideoWriter(filename, codec, self.frame_rate, frame_size)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(e)
            return

        if self.record_video:
            self.video_writer.write(cv_image)
        else:
            # Save as individual images
            filename = 'image_' + str(rospy.get_time()) + '.jpg'  # Unique filename based on timestamp
            cv2.imwrite(filename, cv_image)

    def spin(self):
        rospy.spin()

    def __del__(self):
        if self.record_video and self.video_writer:
            self.video_writer.release()

if __name__ == '__main__':
    try:
        node = ImageRecorder()
        node.spin()
    except rospy.ROSInterruptException:
        pass
