project(bag_synchronizer)

find_package(catkin REQUIRED
    COMPONENTS message_filters sensor_msgs cv_bridge)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV REQUIRED)
  catkin_package(
   CATKIN_DEPENDS message_filters sensor_msgs
  )

add_executable(bag_synchronizer src/main.cpp)
target_link_libraries(bag_synchronizer ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                                        ${OpenCV_LIBS})
