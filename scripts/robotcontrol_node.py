#!/usr/bin/env python
import rospy
import sys
import time
import util
import math
import numpy as np


from sensor_msgs.msg import (Temperature, LaserScan)
from geometry_msgs.msg import (Pose,PoseStamped, PoseWithCovarianceStamped,PoseWithCovariance, Quaternion, Transform, TransformStamped, Twist)
from nav_msgs.msg import (OccupancyGrid, Path, Odometry)
from std_msgs.msg import Int8MultiArray
from tf import (TransformBroadcaster)
from tf.msg import tfMessage


class RobotController(object):
    def __init__(self):
        self.TO_TARGET = 0
        self.GOING_AROUND_OBSTACLE = 1
        self.TRYING_TO_ESCAPE = 2
        self.TARGET_REACHED = 3
        self.min_temp = 0

        try:
            self.map = rospy.wait_for_message("/map", OccupancyGrid, 20)

            rospy.loginfo("Map received. %d X %d, %f px/m." %
                          (self.map.info.width, self.map.info.height,
                           self.map.info.resolution))
            rospy.loginfo("Robot control is working")

        except rospy.exceptions.ROSException:
            rospy.logerr("Problem getting a map or a fireMap. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        try:
            self.pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, 20)

            rospy.loginfo("Pose received. ")
        except rospy.exceptions.ROSException:
            rospy.logerr("Problem getting a pose" )
            sys.exit(1)
        rospy.loginfo("Robot control is working")
        self.noPath = False
        self.fire_width = 3
        self.temperatures = [0,0,0,0]
        self.estimated_pose = Pose()
        self.estimated_fire_map_data = []
        self.target_position = [10,10]
        self.navigation_mode = self.TARGET_REACHED

        def update_temperature(data):
            self.temperatures = data.data
            self.optimal_policy()
            print(self.temperatures)
        def update_pose(data):
            self.estimated_pose = data



        self.temperatureSubscriber = rospy.Subscriber("temperatures", Int8MultiArray, update_temperature)
        self.movementPubliser = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.amclPoseSubscriber = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped, update_pose)
        self.mapPublisher = rospy.Publisher("mapeuh", OccupancyGrid, queue_size=2)

        self.optimal_policy()

    def optimal_policy(self):
        FORWARD = [1,0]
        BACKWARD = [-1,0]
        RIGHT = [0,-1]
        LEFT = [0,1]
        print(self.temperatures)
        if self.temperatures[0] > self.min_temp: #front greater than 50
            cell_pos = util.getCellPosFromRobotPos(self.estimated_pose.pose.pose, FORWARD, self.map.info.resolution)
            cflat = cell_pos[0] + cell_pos[1] * self.map.info.width
            self.estimated_fire_map_data.append(cflat)

        if self.temperatures[1] > self.min_temp: #right greater than 50
            cell_pos = util.getCellPosFromRobotPos(self.estimated_pose.pose.pose, RIGHT, self.map.info.resolution)
            cflat = cell_pos[0] + cell_pos[1] * self.map.info.width
            self.estimated_fire_map_data.append(cflat)

        if self.temperatures[2] > self.min_temp: #back greater than 50
            cell_pos = util.getCellPosFromRobotPos(self.estimated_pose.pose.pose, BACKWARD, self.map.info.resolution)
            cflat = cell_pos[0] + cell_pos[1] * self.map.info.width
            self.estimated_fire_map_data.append(cflat)

        if self.temperatures[3] > self.min_temp: #left greater than 50
            cell_pos = util.getCellPosFromRobotPos(self.estimated_pose.pose.pose, LEFT, self.map.info.resolution)
            cflat = cell_pos[0] + cell_pos[1] * self.map.info.width
            self.estimated_fire_map_data.append(cflat)

        estimated_map = util.combineOccupancyGridWithData(self.map, self.estimated_fire_map_data)
        self.mapPublisher.publish(estimated_map)

#The transition model for all states



def update(node):
    while not rospy.is_shutdown():
            rospy.sleep(10)





if __name__ == '__main__':
    rospy.init_node("robot_control")
    node = RobotController()
    try:
	  update(node)
    except rospy.ROSInterruptException:
	  pass
