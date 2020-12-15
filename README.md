# IntelligentRoboticsProject

The final project for MsC intelligent Robotics.

The goal of this project is to develop a robot model that can safely navigate to a given point in a building by avoiding areas with high temperatures. If the robot reaches the point, then a safe path exists that firefighters can use to get to the location in the building. This will involve using path planning to allow the robot to navigate to a given point in the map, with a heuristic of the temperature readings applied to local navigation decisions. 


Launching the Simulation:

Run...
  1. 'sudo apt-get install ros-melodic-amcl
  2. 'sudo apt-get install ros-melodic-move-base
  3. 'roslaunch firerobot firerobot.launch
  4. 'roslaunch firerobot navigation.launch'
  
The simulation can be visualised in RVIZ. Run 'rosrun rviz rviz' to launch.

