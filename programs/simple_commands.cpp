
#include "haptic_wrist/haptic_wrist.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv) {

    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(false);
    hw.run();

    sleep(1);
    hw.hold(true);
    //
    // // haptic_wrist::jp_type desiredPos;
    // // desiredPos << M_PI / 2.0, M_PI / 2.0, M_PI / 2.0;
    // Eigen::Quaterniond currentQuat = hw.getOrientation(); 
    //
    // Eigen::Vector3d axis = Eigen::Vector3d::UnitY();
    //
    // Eigen::Quaterniond desQuat(1.0, 0.0, 0.0, 0.0);
    //
    sleep(1);
    // Eigen::Quaterniond des = hw.getOrientation();

    // hw.setTarget(des);
    hw.setTriggerHaptics(50);

    while (true) {
        std::cout << "position\n" << hw.getPosition() << std::endl;
        Eigen::Quaterniond orientation = hw.getOrientation();
        Eigen::AngleAxisd angleAxis(orientation);
        std::cout << "Orientation Axis: " << angleAxis.axis().transpose() << ", Angle: " << angleAxis.angle()
                  << std::endl;

        sleep(1);
    }

    hw.stop();
    return 0;
}
