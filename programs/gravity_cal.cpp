#include "haptic_wrist/gravity_comp.h"
#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist/kinematics.h"
#include "utils.h"
#include "yaml-cpp/yaml.h"
#include <barrett/units.h>
#include <fstream>
#include <thread>

#include <barrett/standard_main_function.h>
#define NUM_POINTS 2000

void print_usage(char *program_name) {
    printf("Usage: %s [options]\n", program_name);
    printf("Options\n");
    printf("  --enable-last : Include the last joint in calibration. Exlcuded by default since handle COM intersects "
           "axis of rotation\n");
    printf("  --help : Prints this help message\n");
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

template <size_t DOF>
int wam_main(int argc, char **argv, barrett::ProductManager &pm, barrett::systems::Wam<DOF> &wam) {

    bool enable_last = false;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--enable-last") {
            enable_last = true;
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    if (!enable_last) {
        std::cout << "Note: The last link parameters are not estimated, use --help to see how to include."
                  << std::endl;
    }

    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    if (DOF != 4) {
        std::cout << "Only 4Dof supported" << std::endl;
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
    std::vector<jp_type> wam_poses;
    for (size_t i = 0; i < yaml_config["gravitycal"].size(); i++) {
        auto pose_node = yaml_config["gravitycal"][i];
        poses.push_back({pose_node[4].as<double>(), pose_node[5].as<double>(), pose_node[6].as<double>()});
        jp_type wamPose;
        wamPose[0] = pose_node[0].as<double>();
        wamPose[1] = pose_node[1].as<double>();
        wamPose[2] = pose_node[2].as<double>();
        wamPose[3] = pose_node[3].as<double>();
        wam_poses.push_back(wamPose);
    }

    std::vector<haptic_wrist::jp_type> positions;
    std::vector<haptic_wrist::jt_type> torques;
    std::vector<Eigen::Matrix4d> base_to_world;

    std::vector<DHParameter> dh;
    for (size_t i = 0; i < 3; i++) {
        DHParameter dh_param;
        dh_param.alpha_pi = yaml_config["kinematics"]["dh"][i]["alpha_pi"].as<double>();
        dh_param.a = yaml_config["kinematics"]["dh"][i]["a"].as<double>();
        dh_param.d = yaml_config["kinematics"]["dh"][i]["d"].as<double>();
        dh.push_back(dh_param);
    }
    Kinematics kinematics(dh, Eigen::Matrix4d::Identity());

    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(false);
    hw.setPosition({0, 0, 0});
    hw.run();

    auto out_file = boost::filesystem::path(config_dir) / "gravity_cal.yaml";
    std::cout << "\nThis program will overwrite: " << out_file.string() << std::endl;
    if (!confirmContinue()) {
        std::cout << "Program canceled." << std::endl;
        return 1;
    }

    for (size_t i = 0; i < poses.size(); i++) {
        std::cout << "Moving to\n" << wam_poses[i] << std::endl;
        wam.moveTo(wam_poses[i], true);
        auto wamPose = wam.getToolPose();

        base_to_world.push_back(posQuatToTransform(boost::get<0>(wamPose), boost::get<1>(wamPose)));
        std::cout << "Moving to\n" << poses[i] << std::endl;

        hw.moveTo(poses[i]);
        sleep(1);

        Eigen::Matrix<double, NUM_POINTS, 3> jp;
        Eigen::Matrix<double, NUM_POINTS, 3> jt;

        for (int n = 0; n < NUM_POINTS; n++) {
            jp.row(n) = hw.getPosition();
            jt.row(n) = hw.getTorque();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        positions.push_back(jp.colwise().mean());
        torques.push_back(jt.colwise().mean());
    }
    hw.moveTo({0, 0, 0});
    wam.moveHome();
    hw.stop();

    Eigen::MatrixXd nLL(3 * poses.size(), 3 + 2 * poses.size());
    nLL.setZero();
    for (size_t i = 0; i < poses.size(); i++) {
        nLL(3 * i + 0, 3 + 2 * i + 0) = -1.0;
        nLL(3 * i + 1, 3 + 2 * i + 1) = -1.0;
    }

    size_t n = 3;
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
        auto kin = kinematics.eval(positions[i], base_to_world[i]);
        auto grav = GravityComp::computeGravity(kin);
        for (size_t j = 0; j < 3; j++) {
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

    std::array<Eigen::VectorXd, 3> P;
    Eigen::VectorXd b(3 * poses.size());
    b.setZero();

    for (size_t i = 0; i < 3; i++) {
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
    return 0;
}
