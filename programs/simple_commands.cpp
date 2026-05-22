
#include "haptic_wrist/haptic_wrist.h"
#include "haptic_wrist/handle.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv) {

    // haptic_wrist::HapticWrist hw;
    // hw.gravityCompensate(false);
    // hw.run();
    haptic_wrist::Handle handle;

    sleep(1);
    // hw.hold(true);

    haptic_wrist::jp_type desiredPos;
    // desiredPos << 0, -M_PI / 2.0, 0;
    desiredPos << 0, 0, 0;
    // Eigen::Quaterniond currentQuat = hw.getOrientation(); 
    //
    // Eigen::Vector3d axis = Eigen::Vector3d::UnitY();
    //
    // Eigen::Quaterniond desQuat(1.0, 0.0, 0.0, 0.0);
    //
    sleep(1);
    // Eigen::Quaterniond des = hw.getOrientation();

    // hw.setTarget(desiredPos);
    // hw.jointMoveTo(desiredPos);
    // hw.setTriggerHaptics(50);
    // hw.setTriggerHaptics(255);
    // std::cout << "set hap" << std::endl;

    while (true) {
        // std::cout << "position\n" << hw.getPosition() << std::endl;
        // std::cout << "torque\n" << hw.getTorque() << std::endl;
        // Eigen::Quaterniond orientation = hw.getOrientation();
        // Eigen::AngleAxisd angleAxis(orientation);
        // std::cout << "Orientation Axis: " << angleAxis.axis().transpose() << ", Angle: " << angleAxis.angle()
        //           << std::endl;
        handle.poll();
        if (boost::optional<haptic_wrist::handle_type> opt_handle = handle.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle; 

            std::cout << "bumper: " << handle[0] 
                    << " | trigger: " << handle[1] << std::endl;
        }

        sleep(1);
    }

    // hw.stop();
    return 0;
}
