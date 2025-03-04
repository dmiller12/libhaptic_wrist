#include "haptic_wrist/haptic_wrist.h"

#define NUM_POINTS 2000

int main(int argc, char **argv) {

    std::vector<haptic_wrist::jp_type> poses;
    poses.push_back({0, 0, 0});
    poses.push_back({0.5, 0, 0});
    poses.push_back({-0.5, 0, 0});
    std::vector<haptic_wrist::jp_type> positions;
    std::vector<haptic_wrist::jt_type> torques;

    std::vector<DHParameter> dh;
    dh.push_back({-0.5, 0, 0.455});
    dh.push_back({0.5, 0, 0});
    dh.push_back({0, 0, 0});
    Kinematics kinematics(dh, Eigen::Matrix4d::Identity());

    haptic_wrist::HapticWrist hw;
    hw.run();

    for (size_t i = 0; i < poses.size(); i++) {
        hw.move_to(poses[i]);

        Eigen::Matrix<double, NUM_POINTS, 3> jp;
        Eigen::Matrix<double, NUM_POINTS, 3> jt;

        for (int n = 0; n < NUM_POINTS; n++) {

            jp.row(n) = hw.get_position();
            jt.row(n) = hw.get_torque();
            // TODO: add sleep
        }

        positions.push_back(jp.colwise().mean());
		auto mean_jt = jt.colwise().mean();
		Eigen::Vector3d jt_rearranged;
		jt_rearranged(0) = mean_jt(2);
		jt_rearranged(1) = mean_jt(0);
		jt_rearranged(2) = mean_jt(1);
        torques.push_back(jt_rearranged);
    }

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
        auto kin = kinematics.eval(poses[i]);
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
