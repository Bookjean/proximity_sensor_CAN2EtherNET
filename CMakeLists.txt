cmake_minimum_required(VERSION 3.8)
project(ecan_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ethernet_node src/EthernetCommunication.cpp)
ament_target_dependencies(ethernet_node rclcpp std_msgs sensor_msgs)

add_executable(processing_node src/ProximitySensor_pub.cpp)
ament_target_dependencies(processing_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  ethernet_node
  processing_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()