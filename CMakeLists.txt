cmake_minimum_required(VERSION 2.8.3)
project(gastronomous)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  message_generation
  genmsg
)

catkin_python_setup()

add_message_files(DIRECTORY msg
  FILES
  MealUpdate.msg
)

add_service_files(DIRECTORY srv
  FILES
  MoveBy.srv
  Meal.srv
)

#add_action_files(DIRECTORY action
#  FILES
#)

generate_messages(DEPENDENCIES
  std_msgs
  std_srvs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


