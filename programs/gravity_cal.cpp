#include "haptic_wrist/gravity_comp.h"
#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist/kinematics.h"
#include "utils.h"
#include "yaml-cpp/yaml.h"
#include <barrett/units.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include <barrett/standard_main_function.h>
#define NUM_POINTS 2000

void print_usage(char *program_name) {
    printf("Usage: %s [options]\n", program_name);
    printf("Options\n");
    printf("  --enable-last : Include the last joint in calibration. Exlcuded by default since handle COM intersects "
           "axis of rotation\n");
    printf("  --pause-on-passive-transition : Pause/confirm only when passive target changes between consecutive "
           "poses (default for 7-value gravitycal rows)\n");
    printf("  --pause-every-pose : Pause/confirm at every pose\n");
    printf("  --single-direction : Disable bidirectional sampling and use legacy single-pass sampling\n");
    printf("  --approach-offset-rad <value> : Approach offset for bidirectional sampling (default: 0.03 rad)\n");
    printf("  --help : Prints this help message\n");
}

void print_intro_banner() {
    std::cout
        << "\n"
        << "                *** Barrett WAM Gravity Calibration Utility ***\n"
        << "\n"
        << "This utility will calculate cumulative first moment of mass data for each link\n"
        << "of your WAM Arm. This data is used by the gravity compensation routine to\n"
        << "support WAM's weight in gravity. The program will move the WAM to several\n"
        << "predefined positions and take torque measurements at each location.\n"
        << "\n"
        << "The calculations rely on having accurate kinematic information. Consider\n"
        << "performing the zero-calibration procedure before proceeding.\n"
        << "\n"
        << "IMPORTANT: DO NOT TOUCH the WAM during the measurement process, or the\n"
        << "calibration computations will be significantly wrong.\n"
        << std::endl;
}

Eigen::Matrix4d posQuatToTransform(const Eigen::Vector3d &position, const Eigen::Quaterniond &quaternion) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(); // Start with identity matrix

    // Convert quaternion to rotation matrix
    Eigen::Matrix3d R = quaternion.toRotationMatrix().transpose();
    transformation.block<3, 3>(0, 0) = R;

    // Set translation
    transformation.block<3, 1>(0, 3) = -R * position;

    return transformation;
}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d S;
    S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return S;
}

bool confirmContinue() {
    std::string input;
    std::cout << "Continue? [Y/n]: ";
    std::getline(std::cin, input);

    // Convert input to lowercase
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);

    // Accept empty input (defaults to "yes"), "y", or "yes"
    return input.empty() || input == "y" || input == "yes";
}

bool waitForPoseReady(size_t pose_index, bool has_passive_target, double passive_target_rad, double passive_current_rad,
                      bool bidirectional_sampling) {
    std::cout << "\nPose " << (pose_index + 1) << " reached." << std::endl;
    if (has_passive_target) {
        std::cout << "Passive target [rad]: " << passive_target_rad
                  << " (current: " << passive_current_rad << ")" << std::endl;
    }
    if (bidirectional_sampling) {
        std::cout << "Lock passive joint, then press Enter for bidirectional sample ('q' to quit): ";
    } else {
        std::cout << "Lock passive joint, then press Enter to sample ('q' to quit): ";
    }

    std::string input;
    std::getline(std::cin, input);
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);
    return !(input == "q" || input == "quit" || input == "exit");
}

double wrapToPi(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double angularDistance(double a, double b) {
    return std::abs(wrapToPi(a - b));
}

bool isPassiveStateTransition(size_t pose_index, const std::vector<double>& passive_targets) {
    if (pose_index == 0) {
        return true;
    }

    const double current = passive_targets[pose_index];
    const double previous = passive_targets[pose_index - 1];
    if (!std::isfinite(current) || !std::isfinite(previous)) {
        return true;
    }

    constexpr double same_state_tol = 1.0 * M_PI / 180.0;
    return angularDistance(current, previous) > same_state_tol;
}

template <size_t DOF>
int wam_main(int argc, char **argv, barrett::ProductManager &pm, barrett::systems::Wam<DOF> &wam) {

    bool enable_last = false;
    bool pause_on_passive_transition = true;
    bool bidirectional_sampling = true;
    double approach_offset_rad = 0.03;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--enable-last") {
            enable_last = true;
        } else if (arg == "--pause-on-passive-transition") {
            pause_on_passive_transition = true;
        } else if (arg == "--pause-every-pose") {
            pause_on_passive_transition = false;
        } else if (arg == "--single-direction") {
            bidirectional_sampling = false;
        } else if (arg == "--approach-offset-rad") {
            if (i + 1 >= argc) {
                throw std::runtime_error("--approach-offset-rad requires a value.");
            }
            approach_offset_rad = std::stod(argv[++i]);
            if (!std::isfinite(approach_offset_rad) || approach_offset_rad < 0.0) {
                throw std::runtime_error("--approach-offset-rad must be a non-negative finite value.");
            }
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    print_intro_banner();

    if (!enable_last) {
        std::cout << ">>> NOTE: The last link parameters are not estimated. Use --help to include them."
                  << std::endl;
    }

    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    if (DOF != 4) {
        std::cout << ">>> ERROR: Only 4-DOF WAM is currently supported." << std::endl;
        return 1;
    }
    wam.gravityCompensate();

    std::string config_dir = get_config_directory();
    if (config_dir.empty()) {
        throw std::runtime_error("No valid configuration directory found.");
    }

    boost::filesystem::path config_file = boost::filesystem::path(config_dir) / "haptic_wrist.yaml";
    YAML::Node yaml_config = YAML::LoadFile(config_file.string());

    std::vector<haptic_wrist::jp_type> poses;
    std::vector<double> passive_targets;
    std::vector<jp_type> wam_poses;
    for (size_t i = 0; i < yaml_config["gravitycal"].size(); i++) {
        auto pose_node = yaml_config["gravitycal"][i];
        if (!pose_node.IsSequence() || (pose_node.size() != 6 && pose_node.size() != 7)) {
            throw std::runtime_error("Each gravitycal pose must have 6 values (WAM4 + active2) or 7 values "
                                     "(WAM4 + passive + active2).");
        }

        if (pose_node.size() == 7) {
            passive_targets.push_back(pose_node[4].as<double>());
            poses.push_back({pose_node[5].as<double>(), pose_node[6].as<double>()});
        } else {
            passive_targets.push_back(std::numeric_limits<double>::quiet_NaN());
            poses.push_back({pose_node[4].as<double>(), pose_node[5].as<double>()});
        }

        jp_type wamPose;
        wamPose[0] = pose_node[0].as<double>();
        wamPose[1] = pose_node[1].as<double>();
        wamPose[2] = pose_node[2].as<double>();
        wamPose[3] = pose_node[3].as<double>();
        wam_poses.push_back(wamPose);
    }

    std::vector<haptic_wrist::kq_type> positions;
    std::vector<haptic_wrist::jt_type> torques;
    std::vector<Eigen::Matrix4d> base_to_world;

    haptic_wrist::HapticWrist hw;

    haptic_wrist::Kinematics kinematics = hw.getKinematics();
    hw.gravityCompensate(false);
    hw.run();

    auto out_file = boost::filesystem::path(config_dir) / "gravity_cal.yaml";
    std::cout << ">>> This program will overwrite: " << out_file.string() << std::endl;
    if (!confirmContinue()) {
        std::cout << ">>> ERROR: Calibration canceled." << std::endl;
        return 1;
    }

    std::cout << ">>> Sampling mode: "
              << (bidirectional_sampling ? "bidirectional (+/- approach, averaged)" : "single-direction (legacy)")
              << std::endl;
    if (bidirectional_sampling) {
        std::cout << ">>> Approach offset [rad]: " << approach_offset_rad << std::endl;
    }

    hw.hold(true);

    constexpr double bidir_move_vel = 0.05;
    constexpr double bidir_move_accel = 0.05;
    constexpr int settle_after_move_ms = 300;
    auto sampleMean = [&hw]() -> std::pair<haptic_wrist::kq_type, haptic_wrist::jt_type> {
        Eigen::Matrix<double, NUM_POINTS, 2> jp;
        Eigen::Matrix<double, NUM_POINTS, 2> jt;
        Eigen::Matrix<double, NUM_POINTS, 1> passive;

        for (int n = 0; n < NUM_POINTS; n++) {
            jp.row(n) = hw.getPosition();
            jt.row(n) = hw.getTorque();
            passive(n, 0) = hw.getPassivePosition();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        haptic_wrist::kq_type pos_mean;
        const haptic_wrist::jp_type active_mean = jp.colwise().mean();
        pos_mean << passive.col(0).mean(), active_mean(0), active_mean(1);
        return std::make_pair(pos_mean, jt.colwise().mean());
    };

    for (size_t i = 0; i < poses.size(); i++) {
        std::cout << "\nCurrent Pose: " << (i + 1) << " of " << poses.size() << "." << std::endl;
        std::cout << "Current Phase: MOVE_WAM" << std::endl;
        std::cout << "Current Status: Moving to WAM position ..." << std::endl;
        std::cout << "  WAM target: " << wam_poses[i].transpose() << std::endl;
        wam.moveTo(wam_poses[i], true);
        auto wamPose = wam.getToolPose();

        base_to_world.push_back(posQuatToTransform(boost::get<0>(wamPose), boost::get<1>(wamPose)));
        std::cout << "Current Phase: MOVE_WRIST" << std::endl;
        std::cout << "Current Status: Moving to wrist position ..." << std::endl;
        std::cout << "  Wrist active target: " << poses[i].transpose() << std::endl;

        hw.jointMoveTo(poses[i]);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const bool pose_has_passive_target = std::isfinite(passive_targets[i]);
        const double passive_current = hw.getPassivePosition();
        if (pose_has_passive_target) {
            std::cout << "Required passive state for this pose [rad]: " << passive_targets[i]
                      << " (current: " << passive_current << ")" << std::endl;
        }
        const bool pause_for_passive_transition =
            !pause_on_passive_transition || !pose_has_passive_target || isPassiveStateTransition(i, passive_targets);

        if (pause_for_passive_transition) {
            if (!waitForPoseReady(i, pose_has_passive_target, passive_targets[i], passive_current,
                                  bidirectional_sampling)) {
                std::cout << ">>> ERROR: Calibration canceled by user." << std::endl;
                hw.jointMoveTo(hw.getHome());
                wam.moveHome();
                hw.stop();
                return 1;
            }

            if (pose_has_passive_target) {
                constexpr double passive_tol = 5.0 * M_PI / 180.0;
                constexpr int max_wait_ms = 3000;
                auto wait_start = std::chrono::steady_clock::now();
                bool in_tolerance = false;

                while (std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - wait_start)
                           .count() < max_wait_ms) {
                    const double passive_now = hw.getPassivePosition();
                    if (angularDistance(passive_now, passive_targets[i]) <= passive_tol) {
                        in_tolerance = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }

                if (!in_tolerance) {
                    std::cout << ">>> WARNING: Passive joint not within tolerance at pose " << (i + 1)
                              << ". target=" << passive_targets[i] << " measured=" << hw.getPassivePosition()
                              << " tol=" << passive_tol << std::endl;
                }
            }
        } else {
            std::cout << "Current Status: Pose " << (i + 1) << " uses same passive state as previous pose; "
                      << "sampling without additional pause." << std::endl;
        }

        haptic_wrist::kq_type pos_mean;
        haptic_wrist::jt_type torque_mean;
        if (!bidirectional_sampling) {
            std::tie(pos_mean, torque_mean) = sampleMean();
        } else {
            haptic_wrist::kq_type pos_accum = haptic_wrist::kq_type::Zero();
            haptic_wrist::jt_type torque_accum = haptic_wrist::jt_type::Zero();

            for (int pass = 0; pass < 2; pass++) {
                const double sign = (pass == 0) ? 1.0 : -1.0;
                haptic_wrist::jp_type approach_pose = poses[i];
                approach_pose.array() += sign * approach_offset_rad;

                std::cout << "Current Phase: MEAS_PASS_" << (pass + 1) << std::endl;
                std::cout << "Current Status: Starting a measurement pass (" << (pass + 1)
                          << "/2), approach from " << ((sign > 0.0) ? "+offset" : "-offset") << " ..." << std::endl;

                hw.jointMoveTo(approach_pose, bidir_move_vel, bidir_move_accel);
                hw.jointMoveTo(poses[i], bidir_move_vel, bidir_move_accel);
                std::this_thread::sleep_for(std::chrono::milliseconds(settle_after_move_ms));

                haptic_wrist::kq_type pass_pos_mean;
                haptic_wrist::jt_type pass_torque_mean;
                std::tie(pass_pos_mean, pass_torque_mean) = sampleMean();
                pos_accum += pass_pos_mean;
                torque_accum += pass_torque_mean;
            }

            pos_mean = 0.5 * pos_accum;
            torque_mean = 0.5 * torque_accum;
        }

        positions.push_back(pos_mean);
        torques.push_back(torque_mean);

        std::cout << "Recent Statistics:" << std::endl;
        std::cout << "  Positions: " << pos_mean.transpose() << std::endl;
        std::cout << "    Torques: " << torque_mean.transpose() << std::endl;
    }

    std::cout << ">>> Calibration completed!" << std::endl;
    std::cout << ">>> Moving back to home position." << std::endl;
    hw.jointMoveTo(hw.getHome());
    wam.moveHome();
    hw.stop();

    Eigen::MatrixXd nLL(3 * poses.size(), 3 + 2 * poses.size());
    nLL.setZero();
    for (size_t i = 0; i < poses.size(); i++) {
        nLL(3 * i + 0, 3 + 2 * i + 0) = -1.0;
        nLL(3 * i + 1, 3 + 2 * i + 1) = -1.0;
    }

    size_t n = 2;
    std::vector<Eigen::VectorXd> Y(n);
    std::vector<Eigen::MatrixXd> GT(n);
    for (size_t i = 0; i < n; i++) {
        GT[i].resize(3 * poses.size(), 3 + 2 * poses.size());
        GT[i].setZero();
        Y[i].resize(3 * poses.size());
        Y[i].setZero();
    }

    for (size_t i = 0; i < poses.size(); i++) {
        // need gravity vector for each joint
        auto kin_full = kinematics.eval(positions[i], base_to_world[i]);
        std::array<haptic_wrist::Kin, 3> kin = {kin_full[1], kin_full[2], kin_full[3]};
        auto grav = haptic_wrist::GravityComp::computeGravity(kin);
        for (size_t j = 0; j < n; j++) {
            // grav skew matrix
            GT[j].block<3, 3>(3 * i, 0) = skewSymmetric(grav[j]);
            // GT: -R*L,
            GT[j].block<3, 2>(3 * i, 3 + 2 * i) = -kin[j].to_prev_frame.block<2, 3>(0, 0).transpose();
            // Y
            Y[j].block<3, 1>(3 * i, 0) = torques[i](j) * kin[j].to_prev_frame.block<1, 3>(2, 0);
            if (j < n - 1) {
                Y[j](3 * i + 2) -= torques[i](j + 1);
            } else {
                if (!enable_last) {
                    Y[j](3 * i + 0) = 0;
                    Y[j](3 * i + 1) = 0;
                    Y[j](3 * i + 2) = 0;
                }
            }
        }
    }

    std::array<Eigen::VectorXd, 2> P;
    Eigen::VectorXd b(3 * poses.size());
    b.setZero();

    for (size_t i = 0; i < n; i++) {
        P[i].resize(3 + 2 * poses.size());
        P[i].setZero();
    }
    int last_idx;
    if (enable_last) {
        last_idx = n - 1;
    } else {
        last_idx = n - 2;
    }

    double lambda = 1e-6;
    for (int j = last_idx; j >= 0; j--) {
        b = Y[j];

        if (j < n - 1) {
            b += nLL * P[j + 1];
        }
        double b_norm = b.norm();
        double GT_norm = GT[j].norm();

        Eigen::VectorXd b_scaled = b / b_norm;
        Eigen::MatrixXd GT_scaled = GT[j] / GT_norm;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(GT_scaled, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tol = 1e-6 * svd.singularValues().maxCoeff();

        Eigen::VectorXd invSingularValues = svd.singularValues();
        for (int s = 0; s < svd.singularValues().size(); s++) {
            if (svd.singularValues()(s) > tol) {
                invSingularValues(s) =
                    svd.singularValues()(s) / (svd.singularValues()(s) * svd.singularValues()(s) + lambda * lambda);
            } else {
                invSingularValues(s) = 0.0;
            }
        }

        Eigen::VectorXd scaled_x =
            svd.matrixV() * invSingularValues.asDiagonal() * svd.matrixU().transpose() * b_scaled;
        P[j] = scaled_x * b_norm / GT_norm;
    }

    // write to file
    std::vector<std::vector<double>> outmus;
    for (int j = 0; j < n; j++) {
        std::vector<double> row(P[j].data(), P[j].data() + 3);
        outmus.push_back(row);
        std::cout << "Solution J" << j << ":\n" << P[j].head(3) << std::endl;
    }
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "mus";
    out << YAML::BeginSeq;
    for (const auto &vec : outmus) {
        out << YAML::Flow << vec;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(out_file.string());

    fout << out.c_str() << "\n";
    fout.close();
    std::cout << ">>> Data written to: " << out_file.string() << std::endl;
    return 0;
}
