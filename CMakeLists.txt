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
)

## Library
add_library(${PROJECT_NAME}_lib
	include/${PROJECT_NAME}/ellipse_fitting.h
	src/ellipse_fitting.cpp
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
endif()
