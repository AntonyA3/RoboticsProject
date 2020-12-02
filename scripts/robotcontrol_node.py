#!/usr/bin/env python
import rospy
import sys
import time
from sensor_msgs.msg import Temperature
from geometry_msgs.msg import Pose
from nav_msgs.msg import (OccupancyGrid, Path)
class FireDetectionNode(object):

    def __init__(self):
	#Thermometre is likely to be accurate when it gets to it's true value.
	#The temperature is likely to take time to rise or fall to it's actual tempurature
	
	self.TEMPERATURE_RATE_OF_CHANGE = 0.9
    	self.TEMPERATURE_ACCURACY_PROBABILITY = 0.9
        self.temp = [Temperature(), Temperature(), Temperature(), Temperature()] #tempurature state # temperature at 0:front, 1:right, 2: back, 3: right ,of the robot
	self.pose = Pose(); #pose state
	self.map = OccupancyGrid() #map state
	#self.actions = [Twist(), Twist(), Twist(), Twist()] the set of actions of the robot

	def updateTemperature0(self,data):

	  t0 = self.temp[0].temperature
	  t1 = data.temperature
	  self.temp[0].temperature = self.temp[0].temperature + (t0 - t1) * self.TEMPERATURE_RATE_OF_CHANGE

        def updateTemperature1(self,data):	
	  t0 = self.temp[1].temperature
	  t1 = data.temperature
	  self.temp[1].temperature = self.temp[1].temperature + (t0 - t1) * self.TEMPERATURE_RATE_OF_CHANGE
        def updateTemperature2(self,data): 
	  t0 = self.temp[2].temperature
	  t1 = data.temperature
	  self.temp[2].temperature = self.temp[2].temperature + (t0 - t1) * self.TEMPERATURE_RATE_OF_CHANGE
        def updateTemperature3(self,data):
	  t0 = self.temp[3].temperature
	  t1 = data.temperature
	  self.temp[3].temperature = self.temp[3].temperature + (t0 - t1) * self.TEMPERATURE_RATE_OF_CHANGE

	#path publisher, is likely to be in seperate node
	self.path = Path();
	self.pathPublisher = rospy.Publisher("path", Path, queue_size=10)
	
	#subscribers for each temperature that is detected from surrounding. 
        self.tempSubscriber = [rospy.Subscriber("env_temperature0", Temperature, updateTemperature0), rospy.Subscriber("env_temperature1", Temperature, updateTemperature1), rospy.Subscriber("env_temperature2", Temperature, updateTemperature2), rospy.Subscriber("env_temperature3", Temperature, updateTemperature3)]
	
	
        self.rate = 10.0;
        while not rospy.is_shutdown():

    	    #self.pathPublisher.publish(path)
            #soon to implements actions based on the othe readings


            rospy.loginfo("read temperature")
            if self.rate:
                rospy.sleep(1/self.rate)
            else:
                rospy.sleep(1.0)


	
if __name__ == '__main__':
    rospy.init_node("robot_control")
    rospy.loginfo("Robot control is working")
    try:
	  node = FireDetectionNode()
    except rospy.ROSInterruptException:
	  pass

