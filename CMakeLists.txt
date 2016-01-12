project(bag_synchronizer)

  find_package(catkin REQUIRED
    COMPONENTS message_filters sensor_msgs)
  find_package(PCL 1.7 REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  catkin_package(
   CATKIN_DEPENDS message_filters sensor_msgs
  )

  add_executable(bag_synchronizer src/main.cpp)
  target_link_libraries(bag_synchronizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
