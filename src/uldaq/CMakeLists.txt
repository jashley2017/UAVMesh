cmake_minimum_required(VERSION 3.5)
project(uldaq)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -z defs)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(uldaq_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_subdirectory(src/uldaq) # compile our uldaq library 
# add_executable(buffer_pub src/buffer_pub.cpp)
add_library(buffer_pub_component SHARED src/buffer_pub.cpp)
include_directories(src/uldaq/src include)
target_include_directories(
  buffer_pub_component PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                        "$<INSTALL_INTERFACE:include>")
target_compile_definitions(buffer_pub_component PRIVATE "COMPOSITION_BUILDING_DLL")
# rclcpp_components_register_nodes(buffer_pub_component "uldaq_ros::UldaqPublisher")
rclcpp_components_register_node(
   buffer_pub_component
   PLUGIN "uldaq_ros::UldaqPublisher"
   EXECUTABLE buffer_pub
   )
# this does the same thing as target_link_libraries?
ament_target_dependencies(buffer_pub_component rclcpp rclcpp_components uldaq_msgs) 
target_link_libraries(buffer_pub_component 
  libuldaq
)
install(
  TARGETS buffer_pub_component
  EXPORT buffer_pub_component_export
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_libraries(buffer_pub_component HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components uldaq_msgs) 
ament_package()
