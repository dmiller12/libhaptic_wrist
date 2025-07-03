#include <gtest/gtest.h>
#include "haptic_wrist/kinematics.h"
#include <Eigen/Geometry>

using namespace haptic_wrist;

// Test fixture for Kinematics tests to reuse setup code
class KinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        dh_params = {
            {-0.5, 0.0, 0.465, 0.0},
            {0.5,  0.0, 0.0,   0.0},
            {0.0,  0.0, 0.005, 0.0} 
        };

        world_to_base = Eigen::Matrix4d::Identity();

        Eigen::Matrix4d eef_to_tool;
        eef_to_tool << 0, 0, -1, 0,
                       0, 1, 0, 0,
                       1, 0, 0, 0,
                       0, 0, 0, 1;

        // Instantiate the kinematics object for tests
        kinematics = std::make_unique<Kinematics>(dh_params, eef_to_tool, world_to_base);
    }

    std::vector<DHParameter> dh_params;
    DHParameter toolplate_dh;
    Eigen::Matrix4d world_to_base;
    std::unique_ptr<Kinematics> kinematics;
};

TEST_F(KinematicsTest, ForwardKinematicsAtKnownPositions) {

    // test zero position
    jp_type pos_at_zero = jp_type::Zero();
    std::array<Kin, 4> kin_zero = kinematics->eval(pos_at_zero);
    Eigen::Matrix4d T_tool_zero = kin_zero[3].to_world_frame;

    Eigen::Matrix4d T_expected_zero;
    T_expected_zero << 0, 0, -1, 0,
                       0, 1, 0, 0,
                       1, 0, 0, 0.470,
                       0, 0, 0, 1;

    EXPECT_TRUE(T_tool_zero.isApprox(T_expected_zero, 1e-9));


    // test j1 90 degrees
    jp_type pos_j1_90;
    pos_j1_90 << M_PI / 2.0, 0.0, 0.0;

    std::array<Kin, 4> kin_j1_90 = kinematics->eval(pos_j1_90);
    Eigen::Matrix4d T_tool_j1_90 = kin_j1_90[3].to_world_frame;

    Eigen::Matrix4d T_expected_j1_90;
    T_expected_j1_90 <<  0, -1, 0, 0,
                         0, 0, -1, 0,
                         1, 0, 0, 0.470,
                         0, 0, 0, 1;

    EXPECT_TRUE(T_tool_j1_90.isApprox(T_expected_j1_90, 1e-9));
}
