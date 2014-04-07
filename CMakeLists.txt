cmake_minimum_required(VERSION 2.4.6)
project(trajectory_planner)

find_package(catkin REQUIRED rospy std_msgs pr_msgs std_srvs geometry_msgs graspit_msgs tf_conversions tf staubli_tx60 model_rec2)




catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS std_msgs pr_msgs std_srvs geometry_msgs graspit_msgs tf_conversions tf staubli_tx60 model_rec2
#  DEPENDS system_lib
)

