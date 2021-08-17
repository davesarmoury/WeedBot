#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from weedbot_msgs.msg import Detection
from weedbot_msgs.srv import Localize, PoseListArmCommand, NamedArmCommand
STATUS = "Starting"
HANDLED = True

def move_callback(msg):
    global STATUS, HANDLED
    STATUS = msg.data
    if STATUS == "MOVING":
        HANDLED = False

def Control():
    global STATUS, HANDLED
    rospy.init_node('weedbot_controller', anonymous=True)detection
    pub = rospy.Publisher('/weeds/detection/localized', String, queue_size=10)
    rospy.Subscriber("/weeds/movement", String, move_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if STATUS == "STANDING" and HANDLED == False:
            HANDLED = True

            rospy.wait_for_service('/weeds/move_arm/named')
            named_arm_mover = rospy.ServiceProxy('/weeds/move_arm/named', NamedArmCommand)
            named_arm_mover("CAMERA")

            weeds_detected = rospy.wait_for_message("/weeds/detection/raw", Detection)
            if len(weeds_detected.objects) > 0:
                rospy.wait_for_service('/weeds/localize_weeds')
                localizer = rospy.ServiceProxy('/weeds/localize_weeds', Localize)
                weeds_localized = localize_weeds(weeds_detected)
                named_arm_mover("FLAME ON")

                rospy.wait_for_service('/weeds/move_arm/weeds')
                weeds_arm_mover = rospy.ServiceProxy('/weeds/move_arm/weeds', PoseListArmCommand)
                weeds_arm_mover(weeds_localized)
                named_arm_mover("FLAME OFF")

            named_arm_mover("STOW")

        rate.sleep()

if __name__ == '__main__':
    try:
        Control()
    except rospy.ROSInterruptException:
        pass
