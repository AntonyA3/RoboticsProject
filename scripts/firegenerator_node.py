#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Pose

def getTrueRobotPose(data):
  rospy.loginfo(data)

def getTemperatureAt(pose):
  return [Temperature(), Temperature(), Temperature(), Temperature()]

def listener():
  rate = 10
  temperaturePublisher0 = rospy.Publisher("env_temperature0", Temperature, queue_size=10)
  temperaturePublisher1 = rospy.Publisher("env_temperature1", Temperature, queue_size=10)
  temperaturePublisher2 = rospy.Publisher("env_temperature2", Temperature, queue_size=10)
  temperaturePublisher3 = rospy.Publisher("env_temperature3", Temperature, queue_size=10)

  realRobotPose = Pose()

  temperature0 = Temperature()
  temperature1 = Temperature()
  temperature2 = Temperature()
  temperature3 = Temperature()

  realposesubscriber = rospy.Publisher("base_pose_ground_truth",Odometry, getTrueRobotPose) #used to find true pose

  while not rospy.is_shutdown():

    rospy.loginfo("write temperature")
    temperatures = getTemperatureAt(realRobotPose)
    temperature0 = temperatures[0]
    temperature1 = temperatures[1]
    temperature2 = temperatures[2]
    temperature3 = temperatures[3]
    
    temperaturePublisher0.publish(temperature0)
    temperaturePublisher1.publish(temperature1)
    temperaturePublisher2.publish(temperature2)
    temperaturePublisher3.publish(temperature3)
    
    if rate:
        rospy.sleep(1/rate)
    else:
        rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node("fire_generator")
    rospy.loginfo("Fire is being generated")
    try:
	listener()
    except rospy.ROSInterruptException:
	pass


