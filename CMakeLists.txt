cmake_minimum_required(VERSION 3.8)
project(tuw_path_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  nav2_msgs
  nav2_core
  pluginlib
  Eigen3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/bspline_smoother.cpp src/spline.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(bspline_smoother_node  
  src/standalone_bspline_smoother.cpp src/bspline_smoother_node.cpp src/bspline_smoother.cpp src/spline.cpp
)
target_include_directories(bspline_smoother_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bspline_smoother_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(bspline_smoother_node
  ${dependencies}
)

install(TARGETS bspline_smoother_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
    ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
