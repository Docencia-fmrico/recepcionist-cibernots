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
find_package(nav2_msgs REQUIRED)
find_package(gb_dialog REQUIRED)
find_package(dialogflow_ros2_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sound_play REQUIRED)

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
include_directories(${std_msgs_INCLUDE_DIRS})

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
  nav2_msgs
  gb_dialog
  dialogflow_ros2_interfaces
  std_srvs
  sound_play
)

add_library(${PROJECT_NAME} SHARED
  src/perception_asr/DarknetObjDetectionNode.cpp
  src/perception_asr/ObjectDetectionTo3DfromDepthNode.cpp
  src/tf2_detector/ObjectDetectorImprovedNode.cpp
  src/pid/PIDController.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_library(bt_goTo_node SHARED src/BTS/GoTo.cpp)
add_library(bt_ifChair_node SHARED src/BTS/IfChair.cpp)
add_library(bt_indicateChair_node SHARED src/BTS/IndicateChair.cpp)
add_library(bt_introduce_node SHARED src/BTS/Introduce.cpp)
add_library(bt_searchChair_node SHARED src/BTS/SearchChair.cpp)
add_library(bt_waitPerson_node SHARED src/BTS/WaitPerson.cpp)
add_library(bt_sendChair_node SHARED src/BTS/SendChair.cpp)
add_library(bt_askname_node SHARED src/BTS/AskName.cpp)
add_library(bt_introduceguest_node SHARED src/BTS/IntroduceGuest.cpp)
add_library(bt_askdrink_node SHARED src/BTS/AskDrink.cpp)
add_library(bt_orderdrink_node SHARED src/BTS/OrderDrink.cpp)
add_library(bt_servedrink_node SHARED src/BTS/ServeDrink.cpp)

list(APPEND plugin_libs
  bt_goTo_node
  bt_ifChair_node
  bt_indicateChair_node
  bt_introduce_node
  bt_searchChair_node
  bt_waitPerson_node
  bt_sendChair_node
  bt_askname_node
  bt_introduceguest_node
  bt_askdrink_node
  bt_orderdrink_node
  bt_servedrink_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(darknet_objdetection_tf src/darknet_detection_main.cpp)
ament_target_dependencies(darknet_objdetection_tf ${dependencies})
target_link_libraries(darknet_objdetection_tf ${PROJECT_NAME})

add_executable(recepcionist src/bt_recepcionist.cpp)
ament_target_dependencies(recepcionist ${dependencies})
target_link_libraries(recepcionist ${ZMQ_LIBRARIES})

add_executable(objdetection_2d_to_3d_depth_tf src/detection_2d_to_3d_depth_main.cpp)
ament_target_dependencies(objdetection_2d_to_3d_depth_tf ${dependencies})
target_link_libraries(objdetection_2d_to_3d_depth_tf ${PROJECT_NAME})

add_executable(imageobject_tf src/detector_improved_main.cpp)
ament_target_dependencies(imageobject_tf ${dependencies})
target_link_libraries(imageobject_tf ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  ${plugin_libs}
  recepcionist
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

