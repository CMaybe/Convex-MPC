cmake_minimum_required(VERSION 3.10)

project(convex_mpc VERSION 0.1.0 LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type: Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
endif()


find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
add_library(${PROJECT_NAME} SHARED)

target_include_directories(
	${PROJECT_NAME} PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/include>
)
target_sources(
	${PROJECT_NAME} PRIVATE
	src/convex_mpc.cpp
	src/robot_model.cpp
	src/robot_state.cpp
	src/robot_controller.cpp
)	

target_link_libraries(
	${PROJECT_NAME} PUBLIC
	Eigen3::Eigen
	qpOASES
)

target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)