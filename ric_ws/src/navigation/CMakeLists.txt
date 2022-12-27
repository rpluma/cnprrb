
cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# interface generation must appear before ament_package
# TODO averiguar porqué no genera nada en navigation/srv/resize.hpp
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/Resize.srv"
)

ament_package()

include_directories(
  include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)


add_executable(KeybrdCtrl src/keybrdctrl.cpp)
add_executable(RandomCtrl src/randomctrl.cpp)
add_executable(ReactvCtrl src/reactvctrl.cpp)
add_executable(SquareCtrl src/squarectrl.cpp)
add_executable(actuator src/actuator.cpp)

install(TARGETS
  KeybrdCtrl
  ReactvCtrl
  RandomCtrl
  SquareCtrl
  actuator
  DESTINATION lib/${PROJECT_NAME})



# this must appear after the add_executable section
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
"rosidl_typesupport_cpp")

target_link_libraries(SquareCtrl "${cpp_typesupport_target}")




ament_target_dependencies(KeybrdCtrl rclcpp std_msgs)
ament_target_dependencies(ReactvCtrl rclcpp std_msgs sensor_msgs)
ament_target_dependencies(RandomCtrl rclcpp std_msgs)
ament_target_dependencies(SquareCtrl rclcpp std_msgs)
ament_target_dependencies(actuator rclcpp std_msgs geometry_msgs)

