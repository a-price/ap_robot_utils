cmake_minimum_required(VERSION 2.8.3)
project(ap_robot_utils)
set(BUILD_FLAGS "-std=c++0x -Wreturn-type")
#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "Release")

# Add additional CMake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  image_geometry
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  urdf
  visualization_msgs
  resource_retriever
)
if ($ENV{ROS_DISTRO} STREQUAL "indigo")
  find_package(cmake_modules REQUIRED)
endif ($ENV{ROS_DISTRO} STREQUAL "indigo")


## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# /usr/local/lib/cmake/assimp-3.0/
find_package(ASSIMP REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIR})
link_libraries(${ASSIMP_LIBRARIES})

find_package(CppUnit)
include_directories(${CPPUNIT_INCLUDE_DIR})
link_libraries(${CPPUNIT_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS control_msgs cv_bridge diagnostic_msgs geometry_msgs image_geometry image_transport pcl_ros roscpp sensor_msgs std_msgs tf trajectory_msgs urdf visualization_msgs resource_retriever
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

file(GLOB headers "include/${PROJECT_NAME}/*.h")

file(GLOB felz_source "src/libraries/felzenszwalb/*.h")
include_directories("src/libraries/felzenszwalb")
## Declare a cpp library
add_library(${PROJECT_NAME}
  src/libraries/probability_utils.cpp
  src/libraries/geometry_utils.cpp
  src/libraries/ros_utils.cpp
  src/libraries/pose_conversions.cpp
  src/libraries/image_segmentation.cpp
  src/libraries/SimpleCSVReader.cpp
  src/libraries/DHParams.cpp
  src/libraries/OrientationBins.cpp
  ${headers}
  ${felz_source})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})

## Declare a cpp executable
add_executable(project_robot_mesh src/project_robot_mesh.cpp)
set_target_properties(project_robot_mesh PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})

## Add cmake target dependencies of the executable/library
# add_dependencies(ap_robot_utils_node ap_robot_utils_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(project_robot_mesh ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############
## Mark executables and/or libraries for installation
# install(TARGETS ap_robot_utils ap_robot_utils_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Allow Testing
enable_testing()

# Grab all unit test files
file(GLOB include_source "include/")
list(SORT include_source)
file(GLOB unit_tests_source "test/unit/*.cpp")
list(SORT unit_tests_source)
file(GLOB system_tests_source "test/system/*.cpp")
list(SORT system_tests_source)

# Build full unit test
add_executable(TestAllUnits "test/test_units.cpp" ${unit_tests_source})
add_test(TestAllUnits ${EXECUTABLE_OUTPUT_PATH}/TestAllUnits)
target_link_libraries(TestAllUnits ${PROJECT_NAME})
set_target_properties(TestAllUnits PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
add_dependencies(tests TestAllUnits)

# Build standalone unit tests
message(STATUS "\n-- UNIT TESTS: ")
foreach(utest_src_file ${unit_tests_source})
	get_filename_component(test_base ${utest_src_file} NAME_WE)
	message(STATUS "Adding test ${test_base}" )
	add_executable(${test_base} "test/test_units.cpp" ${utest_src_file})
	add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
	#catkin_add_gtest(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
	target_link_libraries(${test_base} ${PROJECT_NAME})
	set_target_properties(${test_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
	add_dependencies(tests ${test_base})
endforeach(utest_src_file)

# Build system tests
message(STATUS "\n-- System TESTS: ")
foreach(stest_src_file ${system_tests_source})
	get_filename_component(test_base ${stest_src_file} NAME_WE)
	message(STATUS "Adding test ${test_base}" )
	add_executable(${test_base} ${stest_src_file})
	add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
	#catkin_add_gtest(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
	target_link_libraries(${test_base} ${PROJECT_NAME})
	set_target_properties(${test_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
	add_dependencies(tests ${test_base})
endforeach(stest_src_file)
