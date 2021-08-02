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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

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

group.go([0.07766891272901848, 0.6172401800573925, -3.1099726756511736, -1.481179890849294, -0.02139317303660704, -1.0365720710793997, 1.6861259825393777], wait=True)
group.stop()

group.go([0.08559498779817709, 1.1641755704977832, -3.1233108516547263, -1.5473972608252424, -0.040984983911882544, -0.4232918784049007, 1.702083244316969], wait=True)
group.stop()
group.go([0.07766891272901848, 0.6172401800573925, -3.1099726756511736, -1.481179890849294, -0.02139317303660704, -1.0365720710793997, 1.6861259825393777], wait=True)
group.stop()

group.go([0.08559498779817709, 1.1641755704977832, -3.1233108516547263, -1.5473972608252424, -0.040984983911882544, -0.4232918784049007, 1.702083244316969], wait=True)
group.stop()
group.go([0.07766891272901848, 0.6172401800573925, -3.1099726756511736, -1.481179890849294, -0.02139317303660704, -1.0365720710793997, 1.6861259825393777], wait=True)
group.stop()

group.go([0.08559498779817709, 1.1641755704977832, -3.1233108516547263, -1.5473972608252424, -0.040984983911882544, -0.4232918784049007, 1.702083244316969], wait=True)
group.stop()
group.go([0.07766891272901848, 0.6172401800573925, -3.1099726756511736, -1.481179890849294, -0.02139317303660704, -1.0365720710793997, 1.6861259825393777], wait=True)
group.stop()

group.go([0.08559498779817709, 1.1641755704977832, -3.1233108516547263, -1.5473972608252424, -0.040984983911882544, -0.4232918784049007, 1.702083244316969], wait=True)
group.stop()

