# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(firerobot)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

install(DIRECTORY 
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

catkin_install_python(PROGRAMS scripts/robotcontrol_node.py scripts/fire_node.py scripts/firegenerator_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
