#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist/kinematics.h" // We need this to calculate the home orientation
#include "haptic_wrist/trajectory.h" 
#include "utils.h" // For get_config_directory
#include "yaml-cpp/yaml.h"

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <boost/filesystem.hpp>

// Helper function to calculate the robot's true home orientation from DH parameters
Eigen::Quaterniond get_home_orientation() {
    std::string config_dir = get_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("Config directory not found.");
    }

    boost::filesystem::path config_file = boost::filesystem::path(config_dir) / "haptic_wrist.yaml";
    YAML::Node yaml_config = YAML::LoadFile(config_file.string());
    
    // Use the haptic_wrist namespace to access DHParameter
    std::vector<haptic_wrist::DHParameter> dh;
    for (size_t i = 0; i < 3; i++) {
        haptic_wrist::DHParameter dh_param;
        dh_param.alpha_pi = yaml_config["kinematics"]["dh"][i]["alpha_pi"].as<double>();
        dh_param.a = yaml_config["kinematics"]["dh"][i]["a"].as<double>();
        dh_param.d = yaml_config["kinematics"]["dh"][i]["d"].as<double>();
        if (yaml_config["kinematics"]["dh"][i]["theta_pi"]) {
            dh_param.theta_pi = yaml_config["kinematics"]["dh"][i]["theta_pi"].as<double>();
        } else {
            dh_param.theta_pi = 0.0;
        }
        dh.push_back(dh_param);
    }

    // Use the haptic_wrist namespace to access Kinematics
    haptic_wrist::Kinematics kinematics(dh, Eigen::Matrix4d::Identity());
    
    // Evaluate kinematics at the zero position. Use the haptic_wrist namespace for Kin.
    std::array<haptic_wrist::Kin, 3> kin_at_home = kinematics.eval({0.0, 0.0, 0.0});
    
    // The home orientation is the rotation matrix of the final link
    Eigen::Matrix3d home_rotation = kin_at_home[2].to_world_frame.block<3, 3>(0, 0);
    return Eigen::Quaterniond(home_rotation);
}


/**
 * @brief Executes a smooth move to a target orientation.
 */
void execute_smooth_move(haptic_wrist::HapticWrist& wrist, const Eigen::Quaterniond& target_q, double duration) {
    Eigen::Quaterniond start_q = wrist.getOrientation();
    haptic_wrist::Trajectory trajectory(start_q, target_q, duration);

    const int loop_rate_hz = 500;
    const auto loop_period = std::chrono::microseconds(1000000 / loop_rate_hz);
    
    while (!trajectory.is_done()) {
        Eigen::Quaterniond setpoint_q = trajectory.get_setpoint(1.0 / loop_rate_hz);
        wrist.setOrientation(setpoint_q);
        std::this_thread::sleep_for(loop_period);
    }
    // Ensure the final target orientation is set
    wrist.setOrientation(target_q);
}

int main() {
    try {
        haptic_wrist::HapticWrist wrist;

        // Calculate the robot's true home orientation from its DH parameters
        const Eigen::Quaterniond home_orientation = get_home_orientation();
        std::cout << "Robot's true home orientation calculated." << std::endl;

        std::cout << "Starting Haptic Wrist controller..." << std::endl;
        wrist.run();
        wrist.setOrientationGains(8.0, 0.08);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // std::cout << "Moving to true home orientation..." << std::endl;
        // execute_smooth_move(wrist, home_orientation, 2.0);
        // std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // --- Define target rotations *relative to the home orientation* ---
        std::vector<std::pair<std::string, Eigen::Quaterniond>> target_orientations;

        // The end-effector's "pointing" axis is its Z-axis.
        // To point along the base's +Y, we need to rotate the tool's Z to align with the base's Y.
        // This is a -90 degree rotation around the base's X-axis.
        target_orientations.push_back({"Pointing along +Y", home_orientation * Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX()))});
        // target_orientations.push_back({"Pointing along -Y", home_orientation * Eigen::Quaterniond(Eigen::AngleAxisd( M_PI / 2.0, Eigen::Vector3d::UnitX()))});
        // target_orientations.push_back({"Back to Home", home_orientation});

        target_orientations.push_back({"Pointing along +X", home_orientation * Eigen::Quaterniond(Eigen::AngleAxisd( M_PI / 2.0, Eigen::Vector3d::UnitY()))});
        // target_orientations.push_back({"Pointing along -X", home_orientation * Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()))});
        // target_orientations.push_back({"Back to Home", home_orientation});

        // To point along -Z, we can rotate 180 degrees around any perpendicular axis (e.g., Y)
        // target_orientations.push_back({"Pointing along -Z", home_orientation * Eigen::Quaterniond(Eigen::AngleAxisd( -M_PI,       Eigen::Vector3d::UnitY()))});
        // target_orientations.push_back({"Back to Home", home_orientation});

        // --- Execute the sequence with smooth moves ---
        for (const auto& target : target_orientations) {
            std::cout << "\n--- Moving to: " << target.first << " ---" << std::endl;
            execute_smooth_move(wrist, target.second, 1.0);
            
            // Give the controller a moment to settle at the final orientation
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Get and print the final joint positions
            haptic_wrist::jp_type final_positions = wrist.getPosition();
            std::cout << "  > Reached Position [rad]: "
                      << "J1: " << final_positions[0] << ", "
                      << "J2: " << final_positions[1] << ", "
                      << "J3: " << final_positions[2] << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        std::cout << "\nSequence complete. Releasing control." << std::endl;
        wrist.hold(false);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Stopping Haptic Wrist controller." << std::endl;
        wrist.stop();

    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Test finished successfully." << std::endl;
    return 0;
}
