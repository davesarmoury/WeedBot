#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from weedbot_msgs.msg import Detection
from weedbot_msgs.srv import PoseListArmCommand, NamedArmCommand
from geometry_msgs.msg import Pose, PoseArray
import math

STATUS = "Starting"
HANDLED = True

GROUND_TO_ARM = 15.0 * 25.4 / 1000.0
ARM_TO_CAM = 0.1
GROUND_TO_CAM = GROUND_TO_ARM + ARM_TO_CAM
ARM_REACH = 0.5
VERTICAL_DEGREES = 65/1920*1080
IMG_W = 416
IMG_H = 416
DEGREES_PER_PIXEL = VERTICAL_DEGREES / IMG_H
RAD_PER_PIXEL = DEGREES_PER_PIXEL * 0.0174533

def localize_weeds(objects):
    pose_array_msg = PoseArray()
    for weed in objects:
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

    return pose_array_msg

def move_callback(msg):
    global STATUS, HANDLED
    STATUS = msg.data
    if STATUS == "MOVING":
        HANDLED = False

def Control():
    global STATUS, HANDLED
    rospy.init_node('weedbot_controller', anonymous=True)
    pub = rospy.Publisher('/weeds/detection/localized', String, queue_size=10)
    rospy.Subscriber("/weeds/movement", String, move_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if STATUS == "STANDING" and HANDLED == False:
            rospy.loginfo("CHECKING")
            HANDLED = True

            rospy.wait_for_service('/weeds/move_arm/named')
            named_arm_mover = rospy.ServiceProxy('/weeds/move_arm/named', NamedArmCommand)
            named_arm_mover("CAMERA")

#            weeds_detected = rospy.wait_for_message("/weeds/detection/raw", Detection)
#            if len(weeds_detected.objects) > 0:
#                weeds_localized = localize_weeds(weeds_detected)
#                named_arm_mover("FLAME ON")
#
#                rospy.wait_for_service('/weeds/move_arm/weeds')
#                weeds_arm_mover = rospy.ServiceProxy('/weeds/move_arm/weeds', PoseListArmCommand)
#                weeds_arm_mover(weeds_localized)
#                named_arm_mover("FLAME OFF")

            named_arm_mover("STOW")
            rospy.loginfo("DONE")

        rate.sleep()

if __name__ == '__main__':
    try:
        Control()
    except rospy.ROSInterruptException:
        pass
