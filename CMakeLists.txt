cmake_minimum_required(VERSION 3.5)
project(bno055_sensor)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# package up the BNO055_driver as a library
add_library(bno055_driver thirdparty/BNO055_driver/bno055.c)

target_include_directories(bno055_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/BNO055_driver>
    $<INSTALL_INTERFACE:include>)

# the ROS2 class that uses the driver is also a library  
add_library(bno055_sensor
  src/bno055_sensor.cpp
  src/bno055_i2c.c)
rclcpp_components_register_nodes(bno055_sensor "bno055_sensor::BNO055Sensor")

target_include_directories(bno055_sensor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/BNO055_driver>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bno055_sensor
  "rclcpp"
  "rclcpp_components"
  "diagnostic_msgs"
  "sensor_msgs")

target_compile_options(bno055_sensor PRIVATE -Werror)
target_compile_features(bno055_sensor PRIVATE cxx_std_14)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS bno055_sensor bno055_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# build the ROS2 node
add_executable(bno055_sensor_node src/bno055_sensor_node.cpp)
target_include_directories(bno055_sensor_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(bno055_sensor_node bno055_sensor bno055_driver i2c)
target_compile_options(bno055_sensor_node PRIVATE -Werror)
target_compile_features(bno055_sensor PRIVATE cxx_std_14)

install(TARGETS bno055_sensor_node
  EXPORT export_${PROJECT_NAME}  
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  bno055_sensor
)

ament_package()
