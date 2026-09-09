#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include <Eigen/Dense>

#include "simulation_core.hpp"

#include <unistd.h>

namespace {

std::string default_scene_path() {
    char executable[4096];
    const ssize_t size = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
    if (size > 0) {
        executable[size] = '\0';
        std::string directory(executable);
        directory = directory.substr(0, directory.find_last_of('/'));
        const std::string local_scene = directory + "/rsc/anymal_c/scene.xml";
        if (access(local_scene.c_str(), R_OK) == 0) return local_scene;
    }
    return ANYMAL_SCENE_XML;
}

constexpr double kControlDt = 0.001;

}  // namespace

int main(int argc, char** argv) {
    std::string xml_path = default_scene_path();
    bool stand_only = false;
    long max_steps = -1;
    Eigen::Vector3d command = Eigen::Vector3d::Zero();

    for (int index = 1; index < argc; ++index) {
        if (!std::strcmp(argv[index], "--headless")) {
            continue;
        } else if (!std::strcmp(argv[index], "--stand")) {
            stand_only = true;
        } else if (!std::strcmp(argv[index], "--steps") && index + 1 < argc) {
            max_steps = std::atol(argv[++index]);
        } else if (!std::strcmp(argv[index], "--vx") && index + 1 < argc) {
            command[0] = std::atof(argv[++index]);
        } else if (!std::strcmp(argv[index], "--vy") && index + 1 < argc) {
            command[1] = std::atof(argv[++index]);
        } else if (!std::strcmp(argv[index], "--wz") && index + 1 < argc) {
            command[2] = std::atof(argv[++index]);
        } else if (!std::strcmp(argv[index], "--xml") && index + 1 < argc) {
            xml_path = argv[++index];
        } else {
            std::printf(
                "usage: %s [--headless] [--stand] [--steps N] [--vx V] [--vy V] [--wz W] [--xml "
                "PATH]\n",
                argv[0]);
            return 1;
        }
    }

    if (max_steps < 0) max_steps = long(10.0 / kControlDt);

    try {
        ConvexMPC::SimulationCore simulation(xml_path, stand_only);
        simulation.set_velocity_command(command);
        std::printf("running headless for %ld steps (%.1f s)\n", max_steps, max_steps * kControlDt);
        simulation.step(static_cast<int>(max_steps));

        const auto state = simulation.snapshot();
        const bool stable = state.position[2] > 0.3 && state.position[2] < 0.8 && std::abs(state.euler[0]) < 0.5 &&
                            std::abs(state.euler[1]) < 0.5;
        std::printf("%s (position [%.3f %.3f %.3f], rpy [%.3f %.3f %.3f])\n",
                    stable ? "STABLE" : "FALLEN",
                    state.position[0],
                    state.position[1],
                    state.position[2],
                    state.euler[0],
                    state.euler[1],
                    state.euler[2]);
        return stable ? 0 : 1;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
