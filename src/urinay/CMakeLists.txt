cmake_minimum_required(VERSION 3.18)
project(urinay)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(driverless_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  rclcpp
  driverless_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  ament_index_cpp
)

set(URINAY_SOURCES
  src/structures/Circle.cpp
  src/structures/Edge.cpp
  src/structures/Node.cpp
  src/structures/Point.cpp
  src/structures/Trace.cpp
  src/structures/TraceBuffer.cpp
  src/structures/Triangle.cpp
  src/structures/Vector.cpp
  src/modules/DelaunayTri.cpp
  src/modules/Visualization.cpp
  src/modules/WayComputer.cpp
  src/utils/Failsafe.cpp
  src/utils/KDTree.cpp
  src/utils/Logger.cpp
  src/utils/Params.cpp
)

add_executable(${PROJECT_NAME}_exec src/main.cpp)
add_library(${PROJECT_NAME} ${URINAY_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_exec ${dependencies})

install(TARGETS ${PROJECT_NAME}_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
