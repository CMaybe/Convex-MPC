cmake_minimum_required(VERSION 3.16)
project(convex_mpc_wasm LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

FetchContent_Declare(
	eigen
	URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(eigen)

FetchContent_Declare(
	qpoases
	GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
	GIT_TAG releases/3.2.2
)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(qpoases)
target_include_directories(qpOASES SYSTEM INTERFACE ${qpoases_SOURCE_DIR}/include)

FetchContent_Declare(
	mujoco
	GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
	GIT_TAG 3.11.0
)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_WASM_THREADS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(mujoco)
set_target_properties(mujoco_wasm PROPERTIES EXCLUDE_FROM_ALL TRUE)
if(TARGET mujoco_wasm_benchmark)
	set_target_properties(mujoco_wasm_benchmark PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()

add_library(convex_mpc_wasm_core STATIC
	${CMAKE_CURRENT_LIST_DIR}/../simulation/simulation_core.cpp
	${CMAKE_CURRENT_LIST_DIR}/../ConvexMPC/src/convex_mpc.cpp
	${CMAKE_CURRENT_LIST_DIR}/../ConvexMPC/src/robot_model.cpp
	${CMAKE_CURRENT_LIST_DIR}/../ConvexMPC/src/robot_state.cpp
	${CMAKE_CURRENT_LIST_DIR}/../ConvexMPC/src/robot_controller.cpp
)
target_include_directories(convex_mpc_wasm_core PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/../simulation
	${CMAKE_CURRENT_LIST_DIR}/../ConvexMPC/include
)
target_link_libraries(convex_mpc_wasm_core PUBLIC Eigen3::Eigen qpOASES mujoco)

add_executable(convex_mpc_sim_wasm bindings.cpp)
target_link_libraries(convex_mpc_sim_wasm PRIVATE
	convex_mpc_wasm_core
	"-Wl,--whole-archive"
	mujoco
	"-Wl,--no-whole-archive"
)
target_compile_definitions(convex_mpc_sim_wasm PRIVATE CONVEX_MPC_WITH_MUJOCO)
target_compile_options(convex_mpc_sim_wasm PRIVATE -fexceptions)
target_link_options(convex_mpc_sim_wasm PRIVATE
	"--bind"
	"-sMODULARIZE=1"
	"-sEXPORT_ES6=1"
	"-sENVIRONMENT=web,node"
	"-sALLOW_MEMORY_GROWTH=1"
	"-sSTACK_SIZE=5242880"
	"-sDISABLE_EXCEPTION_CATCHING=0"
	"--preload-file"
	"${CMAKE_CURRENT_LIST_DIR}/../simulation/rsc@/rsc"
)
set_target_properties(convex_mpc_sim_wasm PROPERTIES
	SUFFIX ".js"
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../web/public/wasm
)
