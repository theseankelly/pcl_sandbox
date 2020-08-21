cmake_minimum_required(VERSION 3.5)
project(pcl_sandbox)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common io)

include_directories(${PCL_COMMON_INCLUDE_DIRS})

###############################################################################
add_executable(pcl_transformer src/pcl_transformer.cpp)
ament_target_dependencies(
  pcl_transformer
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_eigen"
  "tf2_ros"
  "tf2_geometry_msgs"
  "pcl_conversions"
  "pcl_msgs"
  "pcl_ros"
)
install(TARGETS pcl_transformer
  DESTINATION lib/${PROJECT_NAME})

###############################################################################
add_executable(pcl_converter src/pcl_converter.cpp)
ament_target_dependencies(
  pcl_converter
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_msgs"
)
target_link_libraries(pcl_converter ${PCL_LIBRARIES})
install(TARGETS pcl_converter
  DESTINATION lib/${PROJECT_NAME})

###############################################################################
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

ament_package()
