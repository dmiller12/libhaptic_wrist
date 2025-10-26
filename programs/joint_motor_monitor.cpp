#include "haptic_wrist/haptic_wrist.h"
#include "utils.h"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>
#include <thread>

namespace {

Eigen::Matrix<double, haptic_wrist::kWristDofs, haptic_wrist::kWristDofs> loadJ2MP() {
    const std::string config_file = get_config_directory() + "/haptic_wrist.yaml";
    YAML::Node config = YAML::LoadFile(config_file);
    const YAML::Node j2mp_node = config["j2mp"];

    if (!j2mp_node || !j2mp_node.IsSequence() || j2mp_node.size() != haptic_wrist::kWristDofs) {
        throw std::runtime_error("Invalid or missing j2mp matrix in config: " + config_file);
    }

    Eigen::Matrix<double, haptic_wrist::kWristDofs, haptic_wrist::kWristDofs> matrix;
    for (std::size_t row = 0; row < haptic_wrist::kWristDofs; ++row) {
        const YAML::Node row_node = j2mp_node[row];
        if (!row_node.IsSequence() || row_node.size() != haptic_wrist::kWristDofs) {
            throw std::runtime_error("j2mp row " + std::to_string(row) + " has unexpected format.");
        }
        for (std::size_t col = 0; col < haptic_wrist::kWristDofs; ++col) {
            matrix(row, col) = row_node[col].as<double>();
        }
    }
    return matrix;
}

void printState(const haptic_wrist::jp_type& joints,
                const Eigen::Matrix<double, haptic_wrist::kWristDofs, 1>& motors) {
    std::cout << "Joints (rad): " << joints.transpose() << std::endl;
    std::cout << "Motors (units): " << motors.transpose() << std::endl;
    std::cout << "-----------------------------" << std::endl;
}

} // namespace

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    try {
        auto j2mp = loadJ2MP();

        haptic_wrist::HapticWrist hw;
        hw.gravityCompensate(false);
        hw.run();

        std::cout << "Joint/Motor monitor running. Press Ctrl+C to exit." << std::endl;

        while (true) {
            haptic_wrist::jp_type joints = hw.getPosition();
            Eigen::Matrix<double, haptic_wrist::kWristDofs, 1> motors = j2mp * joints;
            printState(joints, motors);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        hw.stop();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
