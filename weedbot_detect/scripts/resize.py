#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

def callback(img):
    global pub, bridge
    cv_image = bridge.imgmsg_to_cv2(img)
    cv_image = cv_image[0:1080, 400:400+1080]
    cv_image = cv2.resize(cv_image, (416, 416))
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()
    image_message.header.frame_id = img.header.frame_id
    pub.publish(image_message)

def adjust():
    global pub, bridge
    bridge = CvBridge()

    rospy.init_node('adjuster', anonymous=True)

    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    pub = rospy.Publisher('/weeds/detection/image_resized', Image, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        adjust()
    except rospy.ROSInterruptException:
        pass
