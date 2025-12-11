cmake_minimum_required(VERSION 3.8)
project(ecan_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Disable Python interface generation
set(rosidl_generate_interfaces_SKIP_PYTHON_INSTALL TRUE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanFrame.msg"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

add_executable(ethernet_node src/ethernet_communication.cpp)
ament_target_dependencies(ethernet_node rclcpp std_msgs)


rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ethernet_node "${cpp_typesupport_target}")


add_executable(processing_node src/ProximitySensor_pub.cpp)
ament_target_dependencies(processing_node rclcpp std_msgs)


target_link_libraries(processing_node "${cpp_typesupport_target}")


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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()