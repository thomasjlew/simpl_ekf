CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(simpl_ekf)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -O3 -Wall")
#SET(CMAKE_BUILD_TYPE Debug)

### Added Thomas Lew
find_package(cmake_modules REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs cv_bridge tf)
FIND_PACKAGE(Eigen REQUIRED)

CATKIN_PACKAGE(
)

INCLUDE_DIRECTORIES(
   src
   ${Eigen_INCLUDE_DIRS}
)

ADD_EXECUTABLE(simpl_ekf
  src/main.cpp
  ${SOURCEFILES}
)
TARGET_LINK_LIBRARIES(simpl_ekf
  ${catkin_LIBRARIES}
)
