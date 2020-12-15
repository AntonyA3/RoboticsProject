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

        self.noPath = False
        self.temperatures = [0,0,0,0]
        self.estimated_position = [0,0]
        self.estimated_angle = 0
        self.map = OccupancyGrid()
        self.target_position = [10,10]
        self.navigation_mode = self.TARGET_REACHED
        forward_action = Twist();
        forward_action.linear.x = 5;

        left_action = Twist()
        left_action.angular.z = -0.5

        right_action = Twist()
        right_action.angular.z = 0.5

        no_action = Twist()
        no_action.linear.x = 0;


        self.actions = {"forward":forward_action, "left":left_action, "right":right_action, "none":no_action}
        self.last_blocked = {"front": False, "left":False, "right":False}
        self.transition_model = {
            "forward_state":[0.7, 0.1, 0.1, 0.0, 0.1],
            "left_state":[0.1, 0.7, 0.0, 0.1, 0.1 ],
            "right_state":[0.1, 0.0, 0.7, 0.1, 0.1 ],
            "back_state":[0.1, 0.1, 0.1, 0.6, 0.1 ],
            "current_state":[0.05, 0.05, 0.05, 0.05, 0.8 ],
        }

        def update_temperature(data):
            for i in range(4):
                self.temperatures[i] = data.data[i]

        def update_map(data):
            self.map = data


        def update_pose(data):
            self.estimated_position = [data.pose.pose.position.x, data.pose.pose.position.y]
            self.estimated_angle = util.getHeading(data.pose.pose.orientation)

        def update_goal(data):
            self.target_position[0] = data.pose.position.x
            self.target_position[1] = data.pose.position.y
            self.navigation_mode = self.TO_TARGET

        self.temperatureSubscriber = rospy.Subscriber("temperatures", Int8MultiArray, update_temperature)
        self.mapSubscriber = rospy.Subscriber("map", OccupancyGrid, update_map)
        self.movementPubliser = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.amclPoseSubscriber = rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped, update_pose)
        self.goalSubscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, update_goal)
        self.mapPublisher = rospy.Publisher("map", OccupancyGrid, queue_size=2)


    def optimal_policy(self):
        cell = [int(self.estimated_position[0] / self.map.info.resolution) ,int(self.estimated_position[1] / self.map.info.resolution) ]
        target_cell = [int(self.target_position[0] / self.map.info.resolution), int(self.target_position[1] / self.map.info.resolution)]
        forwardVector = [math.cos(self.estimated_angle), math.sin(self.estimated_angle)]
        leftVector = [-math.sin(self.estimated_angle), math.cos(self.estimated_angle)]
        rightVector = [math.sin(self.estimated_angle), -math.cos(self.estimated_angle)]

        #front_blocked = False
        #left_blocked  = False
        #right_blocked = False
        """"
        for i in range(20):
            for j in range(-10, 10, 1):
                c = np.add(cell,np.add(np.multiply(forwardVector, i), np.multiply(rightVector, j)))
                c[0] = int(c[0])
                c[1] = int(c[1])
                cflat = int(c[0] * self.map.info.width + c[1])

                if(cflat >= 0 and cflat < self.map.info.width * self.map.info.height ):
                    if(self.map.data[cflat] > 40):
                        front_blocked = True

                c = np.add(cell,np.add(np.multiply(leftVector, i), np.multiply(forwardVector, j)))
                c[0] = int(c[0])
                c[1] = int(c[1])
                cflat = int(c[0] * self.map.info.width + c[1])

                if(cflat >= 0 and cflat < self.map.info.width * self.map.info.height ):
                    if(self.map.data[cflat] > 40):
                        left_blocked = True


                c = np.add(cell,np.add(np.multiply(rightVector, i), np.multiply(forwardVector, j)))
                c[0] = int(c[0])
                c[1] = int(c[1])
                cflat = int(c[0] * self.map.info.width + c[1])

                if(cflat >= 0 and cflat < self.map.info.width * self.map.info.height ):
                    if(self.map.data[cflat] > 40):
                        right_blocked = True
        """

        #    front_blocked = True
        #if self.temperatures[1] > 50:
        #    left_blocked = True
        #if self.temperatures[2] > 50:
        #    right_blocked = True

        if self.temperatures[0] > 50: #front greater than 50
            for i in range(10,15,1):
                for j in range(-10, 10, 1):
                    c = np.add(cell,np.add(np.multiply(forwardVector, i), np.multiply(rightVector, j)))
                    c[0] = int(c[0])
                    c[1] = int(c[1])
                    cflat = int(c[0] * self.map.info.width + c[1])
                    self.map.data[cflat] = 100

        if self.temperatures[1] > 50: #left greater than 50
            for i in range(10,15,1):
                for j in range(-10, 10, 1):
                    c = np.add(cell,np.add(np.multiply(leftVector, i), np.multiply(forwardVector, j)))
                    c[0] = int(c[0])
                    c[1] = int(c[1])
                    cflat = int(c[0] * self.map.info.width + c[1])
                    self.map.data[cflat] = 100

        if self.temperatures[2] > 50: #right greater than 50
            for i in range(10,15,1):
                for j in range(-10, 10, 1):
                    c = np.add(cell,np.add(np.multiply(rightVector, i), np.multiply(forwardVector, j)))
                    c[0] = int(c[0])
                    c[1] = int(c[1])
                    cflat = int(c[0] * self.map.info.width + c[1])
                    self.map.data[cflat] = 100

        self.mapPublisher.publish(self.map)

        """
        if self.navigation_mode is self.TO_TARGET:
            delta_target = np.subtract(self.target_position, self.estimated_position)
            if not front_blocked:
                action = self.actions["forward"]
                if(abs(delta_target[0]) > abs(delta_target[1])):
                  self.movementPubliser.publish()

                  if(self.target_position[1] > self.estimated_position[1]):
                      action.angular.z = 0.05
                  if(self.target_position[1] < self.estimated_position[1]):
                    action.angular.z = -0.05

                if(abs(delta_target[0]) <= abs(delta_target[1])):

                    if(self.target_position[0] > self.estimated_position[0]):
                      action.angular.z = 0.05
                    if(self.target_position[0] < self.estimated_position[0]):
                      action.angular.z = -0.05
                self.movementPubliser.publish(action)

                print("front not blocked")
            else:
                self.movementPubliser.publish(self.actions["left"])
            if np.sqrt(np.dot(self.target_position, self.estimated_position) * np.dot(self.target_position, self.estimated_position)) < 30:
                print("found")
                self.navigation_mode = self.TARGET_REACHED
                self.movementPubliser.publish(self.actions["none"])

        self.last_blocked["front"] = front_blocked
        self.last_blocked["left"] = left_blocked
        self.last_blocked["right"] = right_blocked
        """

#The transition model for all states



def update(node):
    rate = 10.0;
    while not rospy.is_shutdown():

        node.optimal_policy()
        if rate:
            rospy.sleep(1/rate)
        else:
            rospy.sleep(1.0)




if __name__ == '__main__':
    rospy.init_node("robot_control")
    node = RobotController()
    try:
        node.map = rospy.wait_for_message("/map", OccupancyGrid, 20)

        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (node.map.info.width, node.map.info.height,
                       node.map.info.resolution))
        rospy.loginfo("Robot control is working")

    except rospy.exceptions.ROSException:
        rospy.logerr("Problem getting a map or a fireMap. Check that you have a map_server"
                 " running: rosrun map_server map_server <mapname> " )
        sys.exit(1)

    try:
	  update(node)
    except rospy.ROSInterruptException:
	  pass
