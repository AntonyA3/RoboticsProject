#!/usr/bin/env python
import rospy
import sys
import time
import util
import math
import numpy as np

from sensor_msgs.msg import (Temperature, LaserScan)
from geometry_msgs.msg import (Pose, PoseWithCovarianceStamped,PoseWithCovariance, Twist)
from nav_msgs.msg import (OccupancyGrid, Path, Odometry)
from std_msgs.msg import Int8MultiArray

TEMPERATURE_ACCURACY_PROBABILITY = 0.9
#robots states
temperatures = [0,0,0,0]
estimated_pose = Pose()
map = OccupancyGrid()
target = Pose()
#forward, #turn left, #turn right, #full turn , # stop
forward_action = Twist()
forward_action.linear.x = 50;

left_action = Twist()
left_action.angular.z = -50;
left_action.linear.x = 10;

right_action = Twist();
right_action.angular.z = 100;
left_action.linear.x = 10;

no_action = Twist();
no_action.linear.x = 0;

actions = [forward_action, left_action, right_action, no_action]
last_action = no_action
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
        temperatures[i] = data.data[i]

def update_map(data):
    map = data

def update_pose(data):
    estimated_pose.position = data.pose.pose.position
    estimated_pose.orientation = data.pose.pose.orientation


def update_fire_map(data):
    fire_map = data


temperatureSubscriber = rospy.Subscriber("temperatures", Int8MultiArray, update_temperature)
mapSubscriber = rospy.Subscriber("map", OccupancyGrid, update_map)
fireMapSubscriber = rospy.Subscriber("fire_map", OccupancyGrid, update_fire_map)
movementPubliser = rospy.Publisher("cmd_vel", Twist, queue_size=10)

"""def reward_function(pose, map, tempurature, target, elapsed):
    #four possible next states
    #equal probability of transitioning to each state unless one is blocked
    This is the distance between the current pose and the target pose
    #def heuristic(pose, target){


    reward_result = {
    "forward":1,
    "left":0,
    "right":0,
    "backward":-2,
    "stop":-2
    }
    #0.05 m per cell



    Through SLAM, the robot will know it's estimated location and also the
    occupancy,map, The robot is presumed to need a clearance of 1 meter to
    determine the location that it should move. Therefore the reward will be
    reduced if the robot detects that there are walls along the direction of the
    robot within 0.5 meters.


    cell = [int(pose.position.x / map.info.resolution) ,int(pose.position.y / map.info.resolution)]
    #therefore if the are any occupancies wider than the
    heading = util.getHeading(estimated_pose.orientation)
    forwardVector = [math.cos(heading), math.sin(heading)]
    rightVector = [math.sin(heading), -math.cos(heading)]

    Reward based on distance from target
    #forward
    target_cell = [int(target.position.x / map.info.resolution), int(target.position.y / map.info.resolution)]
    forward_distance = np.sub(target_cell, elapsed * np.add(cell, headingVector))
    forward_distance = np.sqrt(np.dot(forward_distance, forward_distance))

    backwards_distance = np.sub(target_cell, elapsed * np.add(cell, -headingVector))
    backwards_distance = np.sqrt(backwards_distance,backwards_distance)

    right_distance = np.sub(target_cell, elapsed * np.add(cell, rightVector))
    right_distance = np.sqrt(right_distance,right_distance)

    left_distance = np.sub(target_cell, elapsed * np.add(cell, -rightVector))
    left_distance = np.sqrt(left_distance,left_distance)

    distances = [("forward", forward_distance),
    ("left",left_distance),
    ("right",right_distance),
    ("back",backwards_distance)]
    distances = sorted(distances, 1)
    reward_result[distances[1][1]] += 1;
    reward_result[distances[2][1]] -= 1;
    reward_result[distances[3][1]] -= 1;


    The occupancy in a certain direction has the greatest affect on the reward
    value of the action

    in_front = 0
    to_left = 0
    to_right = 0
    to_back = 0
    #occupied if occupancy probability > 40
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

            c = [int(cell[0] + -forwardVector[0] * j + rightVector[0] * i)  ,
            int(cell[1] + forwardVector[1] * j + rightVector[1] * i)]
            trueCell = c[0] * map.info.width + c[1]

            if(trueCell >= 0  and trueCell < map.info.width * map.info.height):
                if(map.data[trueCell] > 40):
                    #blocked location
                    to_back += 1
            else:
                #unknown location
                to_back += 1

            c = [int(cell[0] -rightVector[0] * j + forwardVector[0] * i)  ,
            int(cell[1] -rightVector[1] * j + forwardVector[1] * i)]
            trueCell = c[0] * map.info.width + c[1]

            if(trueCell >= 0  and trueCell < map.info.width * map.info.height):
                if(map.data[trueCell] > 40):
                    #blocked location
                    to_left += 1
            else:
                #unknown location
                to_left += 1

            c = [int(cell[0] + rightVector[0] * j + forwardVector[0] * i)  ,
            int(cell[1] + rightVector[1] * j + forwardVector[1] * i)]
            trueCell = c[0] * map.info.width + c[1]

            if(trueCell >= 0  and trueCell < map.info.width * map.info.height):
                if(map.data[trueCell] > 40):
                    #blocked location
                    to_right+= 1
            else:
                #unknown location
                to_right+= 1


    if (in_front > 0 ):
        reward_result["forward"] = -1
        reward_result["stop"] += 1
    else:
        reward_result["forward"] += 1

    if(to_left > 0):
        reward_result["left"] -= 1
        reward_result["stop"] += 1

    else:
        reward_result["left"] += 1

    if(to_right > 0):
        reward_result["right"] -= 1
        reward_result["stop"] += 1

    else:
        reward_result["right"] += 1
    if(to_back > 0):
        reward_result["back"] -= 1
        reward_result["stop"] += 1

    else:
        reward_result["back"] += 1


    if(temperature[0] > 50):
        reward_result["forward"] -= 5
    if(temperature[1] > 50):
        reward_result["right"] -= 5
    if(temperature[2] > 50):
        reward_result["back"] -= 5
    if(temperature[3] > 50):
        reward_result["left"] -= 5


    return reward_result
            #
"""
def optimal_policy(estimated_pose,estimated_occupancy, estimated_temperature, true_target):
    """
    Through SLAM, the robot will know it's estimated location and also the
    occupancy,map, The robot is presumed to need a clearance of 1 meter to
    determine the location that it should move. Therefore the reward will be
    reduced if the robot detects that there are walls along the direction of the
    robot within 0.5 meters.

    """
    target = true_target
    map = estimated_occupancy
    pose = estimated_pose
    cell = [int(pose.position.x / map.info.resolution) ,int(pose.position.y / map.info.resolution)]
    target_cell = [int(target.position.x / map.info.resolution), int(target.position.y / map.info.resolution)]

    #therefore if the are any occupancies wider than the
    heading = util.getHeading(estimated_pose.orientation)
    forwardVector = [math.cos(heading), math.sin(heading)
    leftVector = [-math.sin(heading), math.cos(heading)]
    rightVector = [math.sin(heading), -math.cos(heading)]

    #rightVector = [math.sin(heading), -math.cos(heading)]
    front_blocked = False
    left_blocked  = False
    right_blocked = False
    for i in range(20):
        c = np.add(cell, np.multiply(forwardVector, i))
        cflat = int(c[0] * map.info.width + c[1])

        if(cflat >= 0 and cflat < map.info.width * map.info.height ):
            if(map.data[cflat] > 40):
                front_blocked = True

        c = np.add(cell, np.multiply(leftVector, i))
        if(cflat >= 0 and cflat < map.info.width * map.info.height ):
            if(map.data[cflat] > 40):
                left_blocked = True

        c = np.add(cell, np.multiply(rightVector, i))
        if(cflat >= 0 and cflat < map.info.width * map.info.height ):
            if(map.data[cflat] > 40):
                right_blocked = True

    if(front_blocked and left_blocked and right_blocked):
        movementPubliser.publish(actions[1])
    elif(front_blocked and left_blocked):
        movementPubliser.publish(actions[2])
    elif (front_blocked and right_blocked):
        movementPubliser.publish(actions[1])

    elif front_blocked:
        print("don't go forward");
        delta_target = np.subtract(target_cell, cell);
        if(headingVector.y >= 0):
            if(delta_target[0] > 0):
                movementPubliser.publish(actions[2])
            else:
                movementPubliser.publish(actions[1])
        elif(headingVector.y < 0):
            if(delta_target[0] > 0):
                movementPubliser.publish(actions[1])
            else:
                movementPubliser.publish(actions[2])

    if not front_blocked:
        movementPubliser.publish(actions[0])


def update():
    rate = 10.0;
    while not rospy.is_shutdown():
        optimal_policy(estimated_pose, map, temperatures,target )
        print("something")
        if rate:
            rospy.sleep(1/rate)
        else:
            rospy.sleep(1.0)
    movementPubliser.publish(actions[3])




if __name__ == '__main__':
    rospy.init_node("robot_control")

    try:
        map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
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
