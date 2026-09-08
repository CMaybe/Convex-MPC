add_library(convex_mpc STATIC
    src/convex_mpc.cpp
    src/robot_model.cpp
    src/robot_state.cpp
    src/robot_controller.cpp
)

target_include_directories(convex_mpc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(convex_mpc PUBLIC
    Eigen3::Eigen
    qpOASES
)

target_compile_options(convex_mpc PRIVATE -Wall -Wextra)
