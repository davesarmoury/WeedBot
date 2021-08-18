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
from weedbot_msgs.srv import NamedArmCommand, NamedArmCommandResponse, PoseListArmCommand, PoseListArmCommandResponse

WEED_POUNCE_HEIGHT = 0.25
WEED_MURDER_HEIGHT = 0.1

J_POSES = {}
J_POSES["CAMERA"] = [[0.0, 0.785, 0.0, 1.5707, 0.0, 0.785, -1.5707]]
J_POSES["STOW"] = [[0.0, -0.5291408138267544, 0.0, 2.3559462835787497, 0.0, 1.316164171279423, -1.5707]]
J_POSES["FLAME ON"] = [[0.0, 0.785, 0.0, 1.5707, 0.0, 0.785, -1.5707], [0.49773661851545764, 1.212717939793039, 0.32461303816489456, 1.5912675134621177, 0.5156422822360645, 1.663230919806021, -1.2206588203733828], [0.48328500814965625, 1.4318396384914507, 0.2797465302537343, 1.4055674211797644, 0.4478855052815096, 1.6040026166368222, -1.263814813205899], [0.13764460709101273, 1.342381660630708, 0.06953864394446725, 1.7232423254937572, 0.12812875800913134, 1.5326014035990347, -1.5727169985730614], [0.0, 0.785, 0.0, 1.5707, 0.0, 0.785, -1.5707]]
J_POSES["FLAME OFF"] = [[0.0, 0.785, 0.0, 1.5707, 0.0, 0.785, -1.5707], [-0.025133316471514178, 1.3272136266421617, -0.0331733998024637, 1.7715908166616552, -0.046746999246378707, 1.5069588907270974, -1.6843796146545582], [0.04850347465977303, 1.4029344869680576, 0.02036125637084966, 1.7083741634439538, 0.03248065334628457, 1.4953006367391668, -1.6290098324307207], [0.3634553092427275, 1.353111004030419, 0.2303744531208643, 1.5537461037059368, 0.3592979816230593, 1.6067223699000655, -1.3538291254183594], [0.0, 0.785, 0.0, 1.5707, 0.0, 0.785, -1.5707]]

#Yes, services for this kind of process are not even close to best practice.  Lazy Dave
def named_callback(req):
    if req.data in J_POSES:
        for i in J_POSES[req.data]:
            group.go(i, wait=True)
            group.stop()

        return NamedArmCommandResponse("Success")
    else:
        return NamedArmCommandResponse("No pose named: " + str(req.data))

#Yes, services for this kind of process are not even close to best practice.  Lazy Dave
def weeds_callback(req):
    for pose_goal in req.data.poses:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = pose.position.x
        pose_goal.position.y = pose.position.y
        pose_goal.position.z = pose.position.z + WEED_POUNCE_HEIGHT
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = pose.position.x
        pose_goal.position.y = pose.position.y
        pose_goal.position.z = pose.position.z + WEED_MURDER_HEIGHT
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        rospy.sleep(5.0)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = pose.position.x
        pose_goal.position.y = pose.position.y
        pose_goal.position.z = pose.position.z + WEED_POUNCE_HEIGHT
        group.set_pose_target(pose_goal)

        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

def main():
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

    rospy.Service('/weeds/move_arm/named', NamedArmCommand, named_callback)
    rospy.Service('/weeds/move_arm/weeds', PoseListArmCommand, weeds_callback)

    rospy.spin()

#    rate = rospy.Rate(1)
#    while not rospy.is_shutdown():
#        print str(robot.get_current_state().joint_state.position)
#        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
