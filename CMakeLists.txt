cmake_minimum_required(VERSION 2.8.3)
project(trep_quadrotor)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs visualization_msgs sensor_msgs tf)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs sensor_msgs tf
)

