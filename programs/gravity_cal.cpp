#include "haptic_wrist/haptic_wrist.h"
#include <barrett/units.h>

#include <barrett/standard_main_function.h>
#define NUM_POINTS 2000

Eigen::Matrix4d posQuatToTransform(const Eigen::Vector3d &position, const Eigen::Quaterniond &quaternion) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(); // Start with identity matrix

    // Convert quaternion to rotation matrix
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();

    // Set translation
    transformation.block<3, 1>(0, 3) = position;

    return transformation;
}

template <size_t DOF>
int wam_main(int argc, char **argv, barrett::ProductManager &pm, barrett::systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    wam.gravityCompensate();

    std::vector<haptic_wrist::jp_type> poses;
    poses.push_back({0, 0, 0});
    poses.push_back({0, 0, 0});
    poses.push_back({0, M_PI/4.0, 0});
    poses.push_back({0, -M_PI/4.0, 0});
    poses.push_back({0, 0, M_PI/4.0});
    poses.push_back({0, 0, -M_PI/4.0});
    poses.push_back({M_PI/4.0, 0, 0});
    poses.push_back({-M_PI/4.0, 0, 0});
    poses.push_back({-M_PI/4.0, -M_PI/4.0, -M_PI/4.0});
    std::vector<jp_type> wam_poses;
    jp_type wamP1;
    jp_type wamP2;
    jp_type wamP3;
    if (DOF == 4) {

        wamP1[0] = 0;
        wamP1[1] = -M_PI / 2;
        wamP1[2] = 0;
        wamP1[3] = 1.845;

        wamP2[0] = 0;
        wamP2[1] = 0;
        wamP2[2] = 0;
        wamP2[3] = 1.845;

        wamP3[0] = 0;
        wamP3[1] = -M_PI / 2;
        wamP3[2] = -M_PI / 2;
        wamP3[3] = 1.845;
    } else {
        std::cout << "Only 4Dof supported" << std::endl;
        return 1;
    }
    
    wam_poses.push_back(wamP1);
    wam_poses.push_back(wamP3);
    wam_poses.push_back(wamP2);
    wam_poses.push_back(wamP2);
    wam_poses.push_back(wamP2);
    wam_poses.push_back(wamP2);
    wam_poses.push_back(wamP2);
    wam_poses.push_back(wamP2);
    wam_poses.push_back(wamP2);

    assert(poses.size() == wam_poses.size());

    std::vector<haptic_wrist::jp_type> positions;
    std::vector<haptic_wrist::jt_type> torques;
    std::vector<Eigen::Matrix4d> base_to_world;

    std::vector<DHParameter> dh;
    dh.push_back({-0.5, 0, 0.520});
    dh.push_back({0.5, 0, 0});
    dh.push_back({0, 0, 0});
    Kinematics kinematics(dh, Eigen::Matrix4d::Identity());

    haptic_wrist::HapticWrist hw;
    hw.gravity_compensate(false);
    hw.set_position({0, 0, 0});
    hw.run();

    for (size_t i = 0; i < poses.size(); i++) {
        wam.moveTo(wam_poses[i], true);
        auto wamPose = wam.getToolPose();

        base_to_world.push_back(posQuatToTransform(boost::get<0>(wamPose), boost::get<1>(wamPose)));
        std::cout << "Moving to\n" << poses[i] << std::endl;

        hw.moveTo(poses[i]);
        sleep(1);

        Eigen::Matrix<double, NUM_POINTS, 3> jp;
        Eigen::Matrix<double, NUM_POINTS, 3> jt;

        for (int n = 0; n < NUM_POINTS; n++) {

            jp.row(n) = hw.get_position();
            jt.row(n) = hw.get_torque();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        positions.push_back(jp.colwise().mean());
        torques.push_back(jt.colwise().mean());
    }
    hw.moveTo({0, 0, 0});
    wam.moveHome();
    hw.stop();

    Eigen::MatrixXd nLL(3 * poses.size(), 3 + 2 * poses.size());
    for (size_t i = 0; i < poses.size(); i++) {
        nLL(3 * i + 0, 3 + 2 * i + 0) = -1.0;
        nLL(3 * i + 1, 3 + 2 * i + 1) = -1.0;
    }

    size_t n = 3;
    std::vector<Eigen::VectorXd> Y(n);
    std::vector<Eigen::MatrixXd> GT(n);

    for (size_t i = 0; i < poses.size(); i++) {
        // need gravity vector for each joint
        auto kin = kinematics.eval(poses[i], base_to_world[i]);
        auto grav = GravityComp::computeGravity(kin);
        for (size_t j = 0; j < 3; j++) {
            // grav skew matrix
            GT[j].resize(3 * poses.size(), 3 + 2 * poses.size());
            GT[j](3 * i + 0, 1) = -grav[j](2);
            GT[j](3 * i + 0, 2) = grav[j](1);
            GT[j](3 * i + 1, 0) = grav[j](2);
            GT[j](3 * i + 1, 2) = -grav[j](0);
            GT[j](3 * i + 2, 0) = -grav[j](1);
            GT[j](3 * i + 2, 1) = grav[j](0);
            // GT: -R*L
            GT[j](3 * i + 0, 3 + 2 * i + 0) = -kin[j].to_prev_frame(0, 0);
            GT[j](3 * i + 0, 3 + 2 * i + 1) = -kin[j].to_prev_frame(1, 0);
            GT[j](3 * i + 1, 3 + 2 * i + 0) = -kin[j].to_prev_frame(0, 1);
            GT[j](3 * i + 1, 3 + 2 * i + 1) = -kin[j].to_prev_frame(1, 1);
            GT[j](3 * i + 2, 3 + 2 * i + 0) = -kin[j].to_prev_frame(0, 2);
            GT[j](3 * i + 2, 3 + 2 * i + 1) = -kin[j].to_prev_frame(1, 2);
            // Y
            Y[j].resize(3 * poses.size());
            Y[j](3 * i + 0) = torques[i](j) * kin[j].to_prev_frame(2, 0);
            Y[j](3 * i + 1) = torques[i](j) * kin[j].to_prev_frame(2, 1);
            Y[j](3 * i + 2) = torques[i](j) * kin[j].to_prev_frame(2, 2);

            if (j < n - 1) {
                Y[j](3 * i + 2) -= torques[i](j + 1);
            }
        }
    }

    std::array<Eigen::VectorXd, 3> P;
    Eigen::VectorXd b(3 * poses.size());
    for (int j = n - 1; j >= 0; j--) {
        b = Y[j];

        if (j < n - 1) {
            b += nLL * P[j + 1];
        }
        P[j].resize(3 + 2 * poses.size());
        P[j] = GT[j].colPivHouseholderQr().solve(b);
    }

    for (int j = 0; j < n; j++) {
        std::cout << "Solution J" << j << ":\n" << P[j].head(3) << std::endl;
    }
}
