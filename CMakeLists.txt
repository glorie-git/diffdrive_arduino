cmake_minimum_required(VERSION 3.8)
project(led_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  std_msgs
  rclcpp
  geometry_msgs
)

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(serial REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Define your ROS 2 executable
add_executable(led_serial_node src/led_serial.cpp)

# Specify the dependencies of the executable
ament_target_dependencies(led_serial_node rclcpp std_msgs geometry_msgs)

install(TARGETS led_serial_node
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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

# target_link_libraries(led_serial_node PUBLIC serial)
# Link with the LibSerial library
target_link_libraries(led_serial_node /usr/lib/x86_64-linux-gnu/libserial.so)

# Specify where the header files are located
install( DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

target_include_directories(led_serial_node 
  PRIVATE include)

# Install the executable
install(TARGETS led_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  /usr/include/libserial
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
