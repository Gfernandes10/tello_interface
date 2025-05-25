cmake_minimum_required(VERSION 3.5)
project(tello_interface)

cmake_policy(SET CMP0071 NEW)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
qt5_wrap_cpp(MOC_SRCS include/tello_interface/teleop_widget.h)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SineCommand.srv"
)

add_executable(tello_interface_node src/tello_interface_node.cpp src/teleop_widget.cpp ${MOC_SRCS})
add_dependencies(tello_interface_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_include_directories(tello_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/../ros_utils>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tello_interface_node rclcpp geometry_msgs tello_msgs Boost_FILESYSTEM)
target_link_libraries(tello_interface_node Qt5::Widgets Qt5::Core Qt5::Gui)
rosidl_target_interfaces(tello_interface_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS tello_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ 
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY plotjuggler/ 
  DESTINATION share/${PROJECT_NAME}/plotjuggler)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
