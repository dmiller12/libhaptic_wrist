#include "haptic_wrist/haptic_wrist_config.h"
#include <boost/filesystem.hpp>
#include <iostream>

#include "yaml-cpp/yaml.h"

namespace YAML {

template <>
struct convert<MoteusConfig> {
    static bool decode(const Node& node, MoteusConfig& c) {
        c.kd = node["kd"].as<Eigen::Vector2d>();
        if (node["transport_args"]) {
            // c.transport_args = node["transport_args"].as<std::vector<std::string>>();
            c.transport_args = node["transport_args"].as<std::string>();
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
struct convert<HandleConfig> {
    static bool decode(const Node& node, HandleConfig& c) {
        c.center_x = node["center_x"] ? node["center_x"].as<int>() : 785;
        c.center_y = node["center_y"] ? node["center_y"].as<int>() : 800;
        c.deadzone = node["deadzone"] ? node["deadzone"].as<int>() : 40;
        c.trigger_max_pos = node["trigger_max_pos"] ? node["trigger_max_pos"].as<int>() : 203;
        c.trigger_min_pos = node["trigger_min_pos"] ? node["trigger_min_pos"].as<int>() : 45;
        return true;
    }
};

template <>
struct convert<JointPositionControllerConfig> {
    static bool decode(const Node& node, JointPositionControllerConfig& c) {
        c.kp = node["kp"].as<Eigen::Vector2d>();
        c.kd = node["kd"].as<Eigen::Vector2d>();
        return true;
    }
};

template <>
struct convert<OrientationControllerConfig> {
    static bool decode(const Node& node, OrientationControllerConfig& c) {
        c.kp = node["kp"] ? node["kp"].as<double>() : 0.0;
        c.kd = node["kd"] ? node["kd"].as<double>() : 0.0;
        return true;
    }
};

template <>
struct convert<PassiveEncoderConfig> {
    static bool decode(const Node& node, PassiveEncoderConfig& c) {
        c.offset_rad = node["offset_rad"] ? node["offset_rad"].as<double>() : 0.0;
        c.scale = node["scale"] ? node["scale"].as<double>() : 1.0;
        return true;
    }
};

template <>
struct convert<HapticWristConfig> {
    static bool decode(const Node& node, HapticWristConfig& config) {
        config.moteus = node["moteus"].as<MoteusConfig>();
        config.dh_parameters = node["kinematics"]["dh"].as<std::vector<haptic_wrist::DHParameter>>();
        config.eef_to_tool = node["kinematics"]["eef_to_tool"].as<Eigen::Matrix4d>();
        config.j2mp = node["j2mp"].as<Eigen::Matrix2d>();
        config.home_position = node["home"].as<Eigen::Vector2d>();
        config.joint_position_controller = node["joint_position_controller"].as<JointPositionControllerConfig>();
        if (node["orientation_controller"]) {
            config.orientation_controller = node["orientation_controller"].as<OrientationControllerConfig>();
        } else {
            config.orientation_controller = OrientationControllerConfig{};
        }
        if (node["passive_encoder"]) {
            config.passive_encoder = node["passive_encoder"].as<PassiveEncoderConfig>();
        } else {
            config.passive_encoder = PassiveEncoderConfig{};
        }
        if (node["handle"]) {
            config.handle = node["handle"].as<HandleConfig>();
        } else {
            config.handle = HandleConfig{};
        }
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
        config.gravity_mus = gravity_yaml["mus"].as<Eigen::Matrix<double, 2, 3>>();

        return config;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading configuration file: " << e.what() << std::endl;
        throw;
    }
}
