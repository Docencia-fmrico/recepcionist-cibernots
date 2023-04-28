cmake_minimum_required(VERSION 3.8)
project(recepcionist_cibernots)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

include_directories(include)
include_directories(include ${ZMQ_INCLUDE_DIRS})

set(dependencies
  rclcpp
  sensor_msgs
  vision_msgs
  darknet_ros_msgs
  image_transport
  cv_bridge
  OpenCV
  message_filters
  pcl_conversions
  pcl_ros
  image_geometry
  depth_image_proc
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
  control_msgs
  kobuki_ros_interfaces
  std_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/perception_asr/DarknetObjDetectionNode.cpp
  src/perception_asr/ObjectDetectionTo3DfromDepthNode.cpp
  src/tf2_detector/ObjectDetectorImprovedNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# add_library(bt_SearchSeat_node SHARED src/BTS/SearchSeat.cpp)
# add_library(bt_detectobject_node SHARED src/bt_nodes/Detectobject.cpp)
# add_library(bt_reachedobject_node SHARED src/bt_nodes/Reachedobject.cpp)
# add_library(bt_searchobject_node SHARED src/bt_nodes/Searchobject.cpp)

list(APPEND plugin_libs
  # bt_SearchSeat_node
  # bt_detectobject_node
  # bt_reachedobject_node
  # bt_searchobject_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(darknet_objdetection_tf src/darknet_detection_main.cpp)
ament_target_dependencies(darknet_objdetection_tf ${dependencies})
target_link_libraries(darknet_objdetection_tf ${PROJECT_NAME})

# add_executable(recepcionist src/bt_recepcionist.cpp)
# ament_target_dependencies(recepcionist ${dependencies})
# target_link_libraries(recepcionist ${ZMQ_LIBRARIES})

add_executable(objdetection_2d_to_3d_depth_tf src/detection_2d_to_3d_depth_main.cpp)
ament_target_dependencies(objdetection_2d_to_3d_depth_tf ${dependencies})
target_link_libraries(objdetection_2d_to_3d_depth_tf ${PROJECT_NAME})

add_executable(imageobject_tf src/detector_improved_main.cpp)
ament_target_dependencies(imageobject_tf ${dependencies})
target_link_libraries(imageobject_tf ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  ${plugin_libs}
  # recepcionist
  darknet_objdetection_tf
  objdetection_2d_to_3d_depth_tf
  imageobject_tf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()

