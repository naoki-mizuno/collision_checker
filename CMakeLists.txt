cmake_minimum_required(VERSION 3.1.3)
project(collision_checker)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(Boost REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_service_files(
  DIRECTORY srv
  FILES Check.srv SetConfig.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  nav_msgs
)

add_library(collision_checker
  src/collision_checker.cpp include/collision_checker.h
)
target_link_libraries(collision_checker
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(collision_checker_node
  src/main.cpp
)
target_link_libraries(collision_checker_node
  collision_checker
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS collision_checker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
