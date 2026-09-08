cmake_minimum_required(VERSION 3.16)
project(convex_mpc_project VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type: Debug, Release, RelWithDebInfo, MinSizeRel" FORCE)
endif()

include(FetchContent)

# --- Eigen (use the system package when available, otherwise fetch) ----------
find_package(Eigen3 3.4 QUIET)
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "System Eigen3 >= 3.4 not found - fetching Eigen 3.4.0")
    FetchContent_Declare(
        eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    )
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
    endif()
    add_library(eigen_headers INTERFACE)
    target_include_directories(eigen_headers INTERFACE ${eigen_SOURCE_DIR})
    add_library(Eigen3::Eigen ALIAS eigen_headers)
endif()

# --- qpOASES ------------------------------------------------------------------
FetchContent_Declare(
    qpoases
    GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
    GIT_TAG releases/3.2.ANYMAL_SCENE_XML1
)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
# qpOASES ships a 2014-era CMake script; default the policies it leaves unset
# and mute its cmake_minimum_required deprecation warning (third-party noise).
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(qpoases)
set(CMAKE_WARN_DEPRECATED ON CACHE BOOL "" FORCE)
# qpOASES's CMake predates target-based include dirs; attach them for consumers
# (SYSTEM keeps its legacy headers from spamming warnings into our build).
if(TARGET qpOASES)
    target_include_directories(qpOASES SYSTEM INTERFACE ${qpoases_SOURCE_DIR}/include)
endif()

add_subdirectory(ConvexMPC)
add_subdirectory(simulation)
