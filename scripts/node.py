#!/usr/bin/python
import rospy
import sys

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("robot_project")

    rospy.loginfo("Node is working")
    rospy.spin()
