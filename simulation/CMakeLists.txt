# --- MuJoCo (prebuilt SDK, fetched automatically) ------------------------------
set(MUJOCO_VERSION "3.11.0" CACHE STRING "MuJoCo release to download")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(MUJOCO_PLATFORM "linux-x86_64")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(MUJOCO_PLATFORM "linux-aarch64")
else()
    message(FATAL_ERROR
        "Automatic MuJoCo download is only set up for Linux. "
        "Install MuJoCo manually and point MUJOCO_DIR at it, or extend this file.")
endif()

include(FetchContent)
FetchContent_Declare(
    mujoco_sdk
    URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}.tar.gz
)
FetchContent_MakeAvailable(mujoco_sdk)

add_library(mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco PROPERTIES
    IMPORTED_LOCATION ${mujoco_sdk_SOURCE_DIR}/lib/libmujoco.so
    INTERFACE_INCLUDE_DIRECTORIES ${mujoco_sdk_SOURCE_DIR}/include
)

# --- simulation executable ------------------------------------------------------
add_executable(mpc_locomotion main.cpp)

target_link_libraries(mpc_locomotion PRIVATE
    convex_mpc
    mujoco
    Eigen3::Eigen
)

target_compile_definitions(mpc_locomotion PRIVATE
    ANYMAL_SCENE_XML="${CMAKE_CURRENT_SOURCE_DIR}/rsc/anymal_c/scene.xml"
)

# Also keep a copy of the robot model next to the executable; main.cpp prefers
# it over the compile-time source path, so the binary stays usable when the
# tree is mounted at a different path.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Viewer is optional: build it when GLFW development files are present
# (apt install libglfw3-dev). Without them the binary runs headless.
find_package(glfw3 QUIET)
if(TARGET glfw)
    target_compile_definitions(mpc_locomotion PRIVATE MPC_USE_VIEWER)
    target_link_libraries(mpc_locomotion PRIVATE glfw)
    message(STATUS "GLFW found - building with interactive viewer")
else()
    message(STATUS "GLFW not found - building headless only (apt install libglfw3-dev for the viewer)")
endif()

# Keep libmujoco next to the executable and resolve it via $ORIGIN, so the
# binary keeps working when the build tree is mounted or moved to another path
# (e.g. host build reused inside the devcontainer).
add_custom_command(TARGET mpc_locomotion POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${mujoco_sdk_SOURCE_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}
        $<TARGET_FILE_DIR:mpc_locomotion>
)
set_target_properties(mpc_locomotion PROPERTIES
    BUILD_RPATH "\$ORIGIN;${mujoco_sdk_SOURCE_DIR}/lib"
    INSTALL_RPATH "\$ORIGIN"
)
