#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from random import randint
import sys

FireGrid = []
FireLocations = []

# 'FireRedundantLocations' provides locations of pixels that are 'on fire' for which all
# neighbouring pixels are already 'on fire' or 'impassible'.
FireRedundantLocations = []

fireMapRowKey = 602
spreadRate = 5.0
fires = 1
fireTotal = fires
totalPassibleArea = 0
status_1 = 0

headstart = 5000 # determines how far fire spreads before any map info is published
"""
NOTE: The function as defined below frequently make use of global variables; they could
be adjusted if required to insted use only input parameters and returns making them more
universally applicable, if required.
"""


def fire_grid_initialize(data):
    """
    Takes an 'OccupancyGrid' input to create a 'FireGrid' (of type list).
    The FireGrid duplicates the OccupancyGrid's impassible areas (i.e. value == -1)
    and sets all other values to 0 (note that an OccupancyGrid's value reflects
    the probability of the robots position; however a 'FireGrid's' value will reflect the
    tempereture, with 0 being room tempereture and 100 the maximum value).

    :Args:
      |  data (nav_msgs.OccupancyGrid): the OccupancyGrid of the map
    :Return:
      |  none
    """
    global FireGrid
    global FireLocations
    global fireTotal
    global spreadRate
    global fires
    global totalPassibleArea
    global status_1
    if (status_1 == 0):
        for i in data.data:
            if (i == -1):
                FireGrid.append(-1.0)
            else:
                FireGrid.append(0.0)
                totalPassibleArea += 1

        for i in range(fires):
            status = False

            while status == False:
                n = randint(0, len(FireGrid) - 1)
                	#print("n:", n, FireGrid[n])
                if (FireGrid[n] == 0.0):
                    FireGrid[n] = 50
                    FireLocations.append(n)
                    status = True

        print("FireGrid initialised...")
        status_1 = 1
        FireLocations.sort()


def fire_grid_update_basic():
    """
    Updates the FireGrid over time. This implementation ('basic') simply increases the temperature
    of any 'on fire' pixels, and any 'passible' (i.e. non '-1' pixels) adjacent pixels over time.
    The rate of temperature increase will be determined by the spreadRate as given in the
    'fire_grid_initialize' function.

    Any pixel >= 50 will be considered 'on fire' and will increase the temperature of adjacent pixels
    (as well as its own) over time; up to a maximum of 100.

    The locations of pixels that are 'on fire' will be stored in the 'FireLocations' array. These will
    be in the form of index locations to be used in conjunction with the 'FireGrid' array.

    The total number of pixels that are 'on fire' will tracked by the 'fireTotal' variable; which can be
    used to track the total spread of the fire at any point. This can be used in conjuction with the
    'totalPassibleArea' variable to provide a percentage of the totable passible areas of the map that are
    'on fire'

    NOTE: This node will use the 'rospy.rate()' method to track the passage of time. i.e. for a temperature
    increase of one per second (remember that for now the tempereture units are arbitraty, i.e. not celcius
    or farenheit etc.). So, for an increase of 1 unit per second, an option would be to use rospy.rate(10)
    and have the temperature of each applicable pixel increase by 0.1 per iteration.

    :args:
      |  none
    :return:
      |  none
    """
    global FireGrid
    global FireLocations
    global FireRedundantLocations
    global fireMapRowKey
    global fireTotal
    global spreadRate
    for i in FireLocations:
        if (FireGrid[i] < 100):
            FireGrid[i] += (spreadRate / 5)
        if (i not in FireRedundantLocations):
            status = False

            row = i // fireMapRowKey

            NeighbourPixels = [(i - 1), (i + fireMapRowKey), (i + 1), (i - fireMapRowKey)]
            NeighbourPixelRow = []
            NeighbourPixelRow.append(NeighbourPixels[0] // fireMapRowKey)
            NeighbourPixelRow.append((NeighbourPixels[1] // fireMapRowKey) - 1)
            NeighbourPixelRow.append(NeighbourPixels[2] // fireMapRowKey)
            NeighbourPixelRow.append((NeighbourPixels[3] // fireMapRowKey) + 1)

            for i2 in range(4):
                pixel = NeighbourPixels[i2]
                pixelRow = NeighbourPixelRow[i2]
                # print (FireGrid[pixel] != 1)
                # print (pixel not in FireLocations)
                # print (pixelRow == row)
                if (FireGrid[pixel] != -1
                        and pixel not in FireLocations
                        and pixelRow == row):
                    status = True
                    FireGrid[pixel] += (spreadRate / 10)
                    if FireGrid[pixel] >= 50:
                        FireLocations.append(pixel)
                        FireLocations.sort()
                        print("fire spreading!")
            if (status == False):
                FireRedundantLocations.append(i)
    FireRedundantLocations.sort()


def fire_map_node():
    global FireGrid
    global FireLocations
    global fireMapRowKey
    global status_1

    rospy.init_node('fireMapNode', anonymous=True)
    rate = rospy.Rate(500)
    pub = rospy.Publisher("/fire_map", OccupancyGrid)

	## Initialize fire_map
    rospy.loginfo("Waiting for a map...")
    occupancy_map = None
    try:
        occupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (occupancy_map.info.width, occupancy_map.info.height,
                       occupancy_map.info.resolution))
        fire_grid_initialize(occupancy_map)
    except rospy.exceptions.ROSException:
        rospy.logerr("Problem getting a map. Check that you have a map_server"
                 " running: rosrun map_server map_server <mapname> " )
        sys.exit(1)

    while not rospy.is_shutdown():
        if (status_1 == 1):
            for i in range(headstart):
                fire_grid_update_basic()
            status_1 = 2
        if (status_1 == 2):
            fire_grid_update_basic()
            fire_map = OccupancyGrid()
            fire_map.info = occupancy_map.info
            fire_map.header.frame_id = "/map"
            # print(fire_map)
            fire_map.data = [int(x) for x in FireGrid]
            pub.publish(fire_map)
        rate.sleep()


if __name__ == '__main__':

    fire_map_node()
