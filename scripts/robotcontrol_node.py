#!/usr/bin/env python
import rospy
import sys
import time
import util
import math
import numpy as np

from sensor_msgs.msg import (Temperature, LaserScan)
from geometry_msgs.msg import (Pose, PoseWithCovarianceStamped)
from nav_msgs.msg import (OccupancyGrid, Path)
from std_msgs.msg import Int8MultiArray

TEMPERATURE_ACCURACY_PROBABILITY = 0.9
#robots states
temperature = [0,0,0,0]
estimated_pose = Pose()
map = OccupancyGrid()
target = Pose()
fir_map = OccupancyGrid()
actions = ["forward", "left", "right", "back", "stop"]

#The transition model for all states
"""
At any pose of the robot, this transition model contains
the probability of moving to a state in front, to the left,
to the right or behind the robot, given the current state and
a high level representation of an action.
index 0:forward_action
index 1: left_action
index 2: right_action
index 3: back_action
index 4: no_action
"""
transition_model = {
    "forward_state":[0.7, 0.1, 0.1, 0.0, 0.1],
    "left_state":[0.1, 0.7, 0.0, 0.1, 0.1 ],
    "right_state":[0.1, 0.0, 0.7, 0.1, 0.1 ],
    "back_state":[0.1, 0.1, 0.1, 0.6, 0.1 ],
    "current_state":[0.05, 0.05, 0.05, 0.05, 0.8 ],
}
grid_x = 10
grid_y = 10
#temperature callback
def update_temperature(data):
    for i in range(4):
        temperature[i] = data.data[i]

def update_map(data):
    map = data

def update_pose(data):
    estimated_pose.position = data.pose.pose.position
    estimated_pose.orientation = data.pose.pose.orientation

def update_fire_map(data):
    fire_map = data

temperatureSubscriber = rospy.Subscriber("temperatures", Int8MultiArray, update_temperature)
mapSubscriber = rospy.Subscriber("map", OccupancyGrid, update_map)
poseIntialiser = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, update_pose)
fireMapSubscriber = rospy.Subscriber("fire_map", OccupancyGrid, update_fire_map)

def reward_function(pose, map, fire_map, target, action):
    #four possible next states
    #equal probability of transitioning to each state unless one is blocked
    #"""This is the distance between the current pose and the target pose"""
    #def heuristic(pose, target){


    #}

    reward_result = 0
    #0.05 m per cell

    """
    The robot should prioritize moving forward
    """
    if(action == "forward"):
        reward_result  = 1
    elif(action == "left" or action == "right"):
        reward_result = 0
    elif (action == "back"):
        reward_result = -1
    elif (action == "no_action"):
        reward_result == -2
    #check infront

    #there
    #check if cells infront of robot are unoccupied or occupied

    """
    Through SLAM, the robot will know it's estimated location and also the
    occupancy,map, The robot is presumed to need a clearance of 1 meter to
    determine the location that it should move. Therefore the reward will be
    reduced if the robot detects that there are walls along the direction of the
    robot within 0.5 meters.

    """
    cell = [int(pose.position.x / map.info.resolution) ,int(pose.position.y / map.info.resolution)]
    #therefore if the are any occupancies wider than the
    heading = util.getHeading(estimated_pose.orientation)
    forwardVector = [math.cos(heading), math.sin(heading)]
    rightVector = [math.sin(heading), -math.cos(heading)]

    """
    The occupancy in a certain direction has the greatest affect on the reward
    value of the action
    """
    in_front = 0
    #occupied if occupancy probability > 50
    for i in range(-10,10,1):
        for j in range(20):
            c = [int(cell[0] + forwardVector[0] * j + rightVector[0] * i)  ,
            int(cell[1] + forwardVector[1] * j + rightVector[1] * i)]
            trueCell = c[0] * map.info.width + c[1]

            if(trueCell >= 0  and trueCell < map.info.width * map.info.height):
                if(map.data[trueCell] > 40):
                    #blocked location
                    in_front += 1
            else:
                #unknown location
                in_front += 1
                pass
    if (in_front > 0 ):
        reward_result -= in_front

    return reward_result
            #
    #frontOccupied = 0
    #for i in range(-10,10, 1):
    #    for j in range(10):
    #        cij = np.add(cell, np.add(np.multiply(rightVector,i), np.multiply(headingVector, j)))
    #        cij = cij.tolist()
    #        cij[0] = int(cij[0])
    #        cij[1] = int(cij[1])

    #        if(cij[0]*map.info.width + cij[1] < len(map.data)):
    #            locationData = map.data[cij[0]*map.info.width + cij[1]]
    #            if(locationData == -1):
    #                frontOccupied = 0
    #            else:
    #                frontOccupied = locationData

    #        else:
    #            frontOccupied = 0
    # TODO: Reward function based on fire
    # if temperature is greater than 50, the reward is immediately negative

    # TODO: Reward function based on distance from target
    #if the distance from target is greater, then the path, the reward is reduced



def update():
    rate = 10.0;

    while not rospy.is_shutdown():

        reward = reward_function(estimated_pose, map, fire_map, target, "forward")
        print("reward is" + str(reward))
        #rospy.loginfo("read temperature")
        if rate:
            rospy.sleep(1/rate)
        else:
            rospy.sleep(1.0)



if __name__ == '__main__':
    rospy.init_node("robot_control")

    try:
        map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (map.info.width, map.info.height,
                       map.info.resolution))
        fire_map = rospy.wait_for_message("/fire_map", OccupancyGrid, 20)
        rospy.loginfo("Fire Map received. %d X %d, %f px/m." %
                      (map.info.width, map.info.height,
                       map.info.resolution))

        rospy.loginfo("Robot control is working")
    except rospy.exceptions.ROSException:
        rospy.logerr("Problem getting a map or a fireMap. Check that you have a map_server"
                 " running: rosrun map_server map_server <mapname> " )
        sys.exit(1)

    try:
	  update()
    except rospy.ROSInterruptException:
	  pass
