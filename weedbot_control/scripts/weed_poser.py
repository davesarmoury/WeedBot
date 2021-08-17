#!/usr/bin/env python

import rospy
from  nav_msgs.msg import Odometry
from std_msgs.msg import String
from weedbot_msgs.srv import Localize, LocalizeResponse
from geometry_msgs.msg import PoseArray, Pose
import math

GROUND_TO_ARM = 15.0 * 25.4 / 1000.0
ARM_TO_CAM = 0.1
GROUND_TO_CAM = GROUND_TO_ARM + ARM_TO_CAM
ARM_REACH = 0.5
VERTICAL_DEGREES = 65/1920*1080
IMG_W = 416
IMG_H = 416
DEGREES_PER_PIXEL = VERTICAL_DEGREES / IMG_H
RAD_PER_PIXEL = DEGREES_PER_PIXEL * 0.0174533

def handle_localize(req):
    pose_array_msg = PoseArray()
    for weed in req.detection.objects:
        pose_msg = Pose()
        avg_x = ( weed.x1 + weed.x2 ) / 2.0
        avg_y = ( weed.y1 + weed.y2 ) / 2.0
        angle_horizontal = ( avg_x - IMG_W / 2.0 ) * RAD_PER_PIXEL
        angle_vertical = ( avg_y - IMG_H / 2.0 ) * RAD_PER_PIXEL * (-1)
        X = math.tan(angle_vertical) * GROUND_TO_CAM
        Y = math.tan(angle_horizontal) * GROUND_TO_CAM
        pose_msg.position.x = X
        pose_msg.position.Y = Y
        pose_msg.position.Y = GROUND_TO_CAM * (-1)
        pose_array_msg.poses(pose_msg)

    return LocalizeResponse(pose_array_msg)

def Localize():
    rospy.init_node('weed_localizer', anonymous=True)
    rospy.Service('/weeds/localize_weeds', Localize, handle_localize)

    rospy.spin()

if __name__ == '__main__':
    try:
        Localize()
    except rospy.ROSInterruptException:
        pass
