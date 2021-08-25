#!/usr/bin/env python

import rospy
from  nav_msgs.msg import Odometry
from std_msgs.msg import String

STATUS = "STARTING"

def callback(msg):
  global STATUS
  lx = abs(msg.twist.twist.linear.x)
  az = abs(msg.twist.twist.angular.z)

  if max(lx, az) > 0.1:
    STATUS = "MOVING"
  else:
    STATUS = "STANDING"

def motion_watcher():
    global STATUS
    pub = rospy.Publisher('/weeds/movement', String, queue_size=10)
    rospy.init_node('motion_watcher', anonymous=True)

    rospy.Subscriber("/gx5/nav/odom", Odometry, callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if STATUS != "STANDING":
          rospy.loginfo(STATUS)
        pub.publish(STATUS)
        rate.sleep()

if __name__ == '__main__':
    try:
        motion_watcher()
    except rospy.ROSInterruptException:
        pass
