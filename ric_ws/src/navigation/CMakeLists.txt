
cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

# interface generation must appear before ament_package
# TODO averiguar porqué no genera nada en navigation/srv/resize.hpp
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
"srv/Resize.srv"
)


add_executable(rndController src/rndController.cpp)
add_executable(keyController src/keyController.cpp)
add_executable(sqrController src/sqrController.cpp)
add_executable(reactivo src/reactivo.cpp)
add_executable(actuator src/actuator.cpp)


# this must appear after the add_executable section
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
"rosidl_typesupport_cpp")

target_link_libraries(sqrController "${cpp_type_support_target}")


install(TARGETS
  rndController
  keyController
  sqrController
  reactivo
  actuator
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

ament_target_dependencies(rndController rclcpp std_msgs)
ament_target_dependencies(keyController rclcpp std_msgs)
ament_target_dependencies(sqrController rclcpp std_msgs)
ament_target_dependencies(reactivo      rclcpp std_msgs sensor_msgs)
ament_target_dependencies(actuator rclcpp std_msgs geometry_msgs)



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

ament_package()






