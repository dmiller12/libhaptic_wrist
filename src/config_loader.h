#include "haptic_wrist/haptic_wrist_config.h"
#include <boost/filesystem.hpp>
#include <iostream>

#include "yaml-cpp/yaml.h"

namespace YAML {

template <>
struct convert<MoteusConfig> {
    static bool decode(const Node& node, MoteusConfig& c) {
        c.kd = node["kd"].as<Eigen::Vector2d>();

        // fallback to usbfd
        if (node["transport_type"]) {
            c.transport_type = node["transport_type"].as<std::string>();
        } else {
            c.transport_type = "usb"; 
        }

        // Decode specific transport arguments
        if (node["transport_usb"]) {
            c.transport_usb = node["transport_usb"].as<std::string>();
        }
        if (node["transport_pcie"]) {
            c.transport_pcie = node["transport_pcie"].as<std::string>();
        }
        return true;
    }
};
template <>
struct convert<haptic_wrist::DHParameter> {
    static bool decode(const Node& node, haptic_wrist::DHParameter& p) {
        p.alpha_pi = node["alpha_pi"].as<double>();
        p.a = node["a"].as<double>();
        p.d = node["d"].as<double>();
        // theta_pi is optional in legacy configs; default to zero when omitted
        p.theta_pi = node["theta_pi"] ? node["theta_pi"].as<double>() : 0.0;
        return true;
    }
};

template <>
struct convert<JointPositionControllerConfig> {
    static bool decode(const Node& node, JointPositionControllerConfig& c) {
        c.kp = node["kp"].as<Eigen::Vector3d>();
        c.kd = node["kd"].as<Eigen::Vector3d>();
        return true;
    }
};

template <>
struct convert<OrientationControllerConfig> {
    static bool decode(const Node& node, OrientationControllerConfig& c) {
        c.kp = node["kp"].as<double>();
        c.kd = node["kd"].as<double>();
        return true;
    }
};

template <>
struct convert<HapticWristConfig> {
    static bool decode(const Node& node, HapticWristConfig& config) {
        config.moteus = node["moteus"].as<MoteusConfig>();
        config.dh_parameters = node["kinematics"]["dh"].as<std::vector<haptic_wrist::DHParameter>>();
        config.eef_to_tool = node["kinematics"]["eef_to_tool"].as<Eigen::Matrix4d>();
        config.j2mp = node["j2mp"].as<Eigen::Matrix3d>();
        config.home_position = node["home"].as<Eigen::Vector3d>();
        config.joint_position_controller = node["joint_position_controller"].as<JointPositionControllerConfig>();
        config.orientation_controller = node["orientation_controller"].as<OrientationControllerConfig>();
        return true;
    }
};

template <typename T, int Rows>
struct convert<Eigen::Matrix<T, Rows, 1>> {
    static bool decode(const Node& node, Eigen::Matrix<T, Rows, 1>& v) {
        // A vector should be a sequence
        if (!node.IsSequence() || node.size() != Rows) {
            return false;
        }

        // Loop through the flat list and fill the vector
        for (int i = 0; i < Rows; ++i) {
            v(i) = node[i].as<T>();
        }
        return true;
    }
};

// Generic Eigen::Matrix decoder
template <typename T, int Rows, int Cols>
struct convert<Eigen::Matrix<T, Rows, Cols>> {
    static bool decode(const Node& node, Eigen::Matrix<T, Rows, Cols>& m) {
        if (!node.IsSequence() || node.size() != Rows)
            return false;
        for (int i = 0; i < Rows; ++i) {
            const Node& row = node[i];
            if (!row.IsSequence() || row.size() != Cols)
                return false;
            for (int j = 0; j < Cols; ++j) {
                m(i, j) = row[j].as<T>();
            }
        }
        return true;
    }
};
} // namespace YAML

HapticWristConfig load_config(const std::string& config_dir) {
    try {

        boost::filesystem::path main_config_path = boost::filesystem::path(config_dir) / "haptic_wrist.yaml";
        YAML::Node main_yaml = YAML::LoadFile(main_config_path.string());

        HapticWristConfig config = main_yaml.as<HapticWristConfig>();

        boost::filesystem::path gravity_config_path = boost::filesystem::path(config_dir) / "gravity_cal.yaml";
        YAML::Node gravity_yaml = YAML::LoadFile(gravity_config_path.string());
        config.gravity_mus = gravity_yaml["mus"].as<Eigen::Matrix3d>();

        return config;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading configuration file: " << e.what() << std::endl;
        throw;
    }
}
