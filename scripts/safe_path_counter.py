#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int8MultiArray
from util import MapToRealConverter, getHeading
import math

class ThermalCaptor:
    firemap = None
    pose = None
    nb_measures = 0
    nb_burn = 0
    nb_dangerous = 0
    avg_temps = 0

    def __init__(self, thermometer_range_m=1):
        self.thermometer_range_m = thermometer_range_m
        self.converter = MapToRealConverter()
        rospy.init_node('thermal_captor', anonymous=True)
        rate = rospy.Rate(10)
        rospy.Subscriber("/fire_map", OccupancyGrid, self.fire_spread)
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.position_change)

    def position_change(self, data):
        self.pose = data.pose.pose
        self.pose.position.x += int(602*0.025)
        self.pose.position.y += int(602*0.025)

        if self.firemap == None:
            print("Received position, waiting for a firemap to estimate the temperature")
        else:
            self.update_temperature()

    def fire_spread(self, data):
        self.firemap = data
        if self.pose == None:
            print("Received firemap, waiting for a pose to estimate the temperature")
        else:
            self.update_temperature()

    def update_temperature(self):
        temperatures = []
        for i in [(1, 0),(0, -1),(-1, 0),(0, 1), ]:
            robot_angle = getHeading(self.pose.orientation)
            sensor_pos_x = self.pose.position.x + i[0]*math.cos(robot_angle) - i[1]*math.sin(robot_angle)
            sensor_pos_y = self.pose.position.y + i[1]*math.cos(robot_angle) + i[0]*math.sin(robot_angle)
            pixel_nb = self.converter.RealToOccupancyGrid(sensor_pos_x, sensor_pos_y)
            temperatures.append(self.firemap.data[pixel_nb])
        temp = sum(temperature)
        nb_measures += 1
        avg_temps = (avg_temps * (nb_measures-1) + 1 * temp/4)/100
        for i in range(4):
            if temp[i] > 50:
                nb_burn += 1
            if temp[i] > 30:
                nb_dangerous += 1
        print("Avg temperature:", avg_temps)
if __name__ == '__main__':
    captor = ThermalCaptor()

    rospy.spin()
    print("The sensor could have burn", nb_burn, "times")
