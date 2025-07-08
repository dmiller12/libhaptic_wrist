#include <gtest/gtest.h>
#include "haptic_wrist/kinematics.h"
#include <Eigen/Geometry>

using namespace haptic_wrist;

// Test fixture for Kinematics tests to reuse setup code
class KinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        dh_params = {
            {-0.5, 0.0, 0.37, 0.0},
            {0.5, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0},
        };

        world_to_base = Eigen::Matrix4d::Identity();

        Eigen::Matrix4d eef_to_tool;
        eef_to_tool << 0, 0,  1, 0,
                       1.0, 0, 0, 0,
                       0,  1,  0, 0,
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
    
    // int i = 0;
    // for (const Kin& k : kin_zero) {
    //     std::cout << "frame: " << i << std::endl << k.to_world_frame << std::endl;
    //     ++i;
    // }

    Eigen::Matrix4d T_expected_zero;
    T_expected_zero << 0, 0, 1, 0,
                       1, 0, 0, 0,
                       0, 1, 0, 0.370,
                       0, 0, 0, 1;

    EXPECT_TRUE(T_tool_zero.isApprox(T_expected_zero, 1e-9));


    // test j1 90 degrees
    jp_type pos_operator_home;
    pos_operator_home << -M_PI/2.0, -M_PI/2.0, 0.0;

    std::array<Kin, 4> kin_operator_home = kinematics->eval(pos_operator_home);
    // i = 0;
    // for (const Kin& k : kin_operator_home) {
    //     std::cout << "frame: " << i << std::endl << k.to_world_frame << std::endl;
    //     ++i;
    // }
    Eigen::Matrix4d T_tool_operator_hom = kin_operator_home[3].to_world_frame;

    Eigen::Matrix4d T_expected_operator_home;
    T_expected_operator_home <<  1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0.370,
                         0, 0, 0, 1;

    EXPECT_TRUE(T_tool_operator_hom.isApprox(T_expected_operator_home, 1e-9));
}
