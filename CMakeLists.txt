cmake_minimum_required(VERSION 3.12)
project(social_nav_utils)

add_definitions(-std=c++17)

find_package(catkin REQUIRED
	COMPONENTS
		angles
)

find_package(PkgConfig)
find_package(Eigen3 REQUIRED)
# try below if Eigen is not properly found
# pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		${PROJECT_NAME}_lib
	CATKIN_DEPENDS
		# empty
	DEPENDS
		EIGEN3
)

## Library
add_library(${PROJECT_NAME}_lib
	include/${PROJECT_NAME}/ellipse_fitting.h
	src/ellipse_fitting.cpp
	include/${PROJECT_NAME}/gaussians.h
	src/gaussians.cpp
	include/${PROJECT_NAME}/heading_direction_disturbance.h
	src/heading_direction_disturbance.cpp
	include/${PROJECT_NAME}/personal_space_intrusion.h
	src/personal_space_intrusion.cpp
	include/${PROJECT_NAME}/formation_space_intrusion.h
	src/formation_space_intrusion.cpp
	include/${PROJECT_NAME}/passing_speed_comfort.h
	src/passing_speed_comfort.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
)

## Install
install(TARGETS ${PROJECT_NAME}_lib
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Testing
# catkin run_tests --no-deps social_nav_utils --verbose
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(test_ellipse_fitting test/test_ellipse_fitting.cpp)
	if(TARGET test_ellipse_fitting)
		target_link_libraries(test_ellipse_fitting ${PROJECT_NAME}_lib)
	endif()
	catkin_add_gtest(test_gaussians test/test_gaussians.cpp)
	if(TARGET test_gaussians)
		target_link_libraries(test_gaussians ${PROJECT_NAME}_lib)
	endif()
	catkin_add_gtest(test_distance_vector test/test_distance_vector.cpp)
	if(TARGET test_distance_vector)
		target_link_libraries(test_distance_vector ${PROJECT_NAME}_lib)
	endif()
	catkin_add_gtest(test_relative_location test/test_relative_location.cpp)
	if(TARGET test_relative_location)
		target_link_libraries(test_relative_location ${PROJECT_NAME}_lib)
	endif()
	catkin_add_gtest(test_heading_direction_disturbance test/test_heading_direction_disturbance.cpp)
	if(TARGET test_heading_direction_disturbance)
		target_link_libraries(test_heading_direction_disturbance ${PROJECT_NAME}_lib)
	endif()
	catkin_add_gtest(test_personal_space_intrusion test/test_personal_space_intrusion.cpp)
	if(TARGET test_personal_space_intrusion)
		target_link_libraries(test_personal_space_intrusion ${PROJECT_NAME}_lib)
	endif()
	catkin_add_gtest(test_formation_space_intrusion test/test_formation_space_intrusion.cpp)
	if(TARGET test_formation_space_intrusion)
		target_link_libraries(test_formation_space_intrusion ${PROJECT_NAME}_lib)
	endif()
	catkin_add_gtest(test_passing_speed_comfort test/test_passing_speed_comfort.cpp)
	if(TARGET test_passing_speed_comfort)
		target_link_libraries(test_passing_speed_comfort ${PROJECT_NAME}_lib)
	endif()
endif()
