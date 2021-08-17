#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

J_POSES = {}
J_POSES["CAMERA"] = [[0.0, 0.785, 0.0, 1.5707, 0.0, 0.785, -1.5707]]
J_POSES["STOW"] = [[0.0, -0.5291408138267544, 0.0, 2.3559462835787497, 0.0, 1.316164171279423, -1.5707]]
J_POSES["FLAME ON"] = [
[0.0, 0.785, 0.0, 1.5707, 0.0, 0.785, -1.5707],
[-0.0367042187756903, 1.457636082074394, 0.08504282295804119, 1.6726252205512753, -0.036259470873647004, 1.5270838664526085, -1.5579966419637579],
[0.28236947761135334, 1.4930424092928343, 0.2976072864204597, 1.4716469725693009, 0.2701074517147879, 1.6016244137833813, -1.3077852006997617]
]

def main()
    global group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('weed_arm_controller', anonymous=True)

    robot = moveit_commander.RobotCommander()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(0.5)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print str(robot.get_current_state())
        rate.sleep()

    #group.go([0.08559498779817709, 1.1641755704977832, -3.1233108516547263, -1.5473972608252424, -0.040984983911882544, -0.4232918784049007, 1.702083244316969], wait=True)
    #group.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
