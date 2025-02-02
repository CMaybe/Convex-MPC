cmake_minimum_required(VERSION 3.24)
project(mpc_example VERSION 0.1.0 LANGUAGES CXX)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type: Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
endif()

if(NOT CMAKE_PREFIX_PATH)
	set(CMAKE_PREFIX_PATH $ENV{LOCAL_INSTALL} ${CMAKE_PREFIX_PATH})
endif()

find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
find_package(raisim CONFIG REQUIRED)
find_package(convex_mpc REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries (${PROJECT_NAME} PUBLIC 
raisim::raisim
pthread
Eigen3::Eigen
convex_mpc
)
target_compile_options(${PROJECT_NAME} PUBLIC "-march=native")

set(SOURCE_FOLDER "${CMAKE_SOURCE_DIR}/rsc")
set(DEST_FOLDER "${CMAKE_BINARY_DIR}/rsc")
file(COPY ${SOURCE_FOLDER} DESTINATION ${CMAKE_BINARY_DIR})

