#include "haptic_wrist/haptic_wrist.h"
#include <barrett/units.h>

#include <barrett/standard_main_function.h>

Eigen::Matrix4d posQuatToTransform(const Eigen::Vector3d &position, const Eigen::Quaterniond &quaternion) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(); // Start with identity matrix

    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = position;

    return transformation;
}

template <size_t DOF>
int wam_main(int argc, char **argv, barrett::ProductManager &pm, barrett::systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    wam.gravityCompensate();

    haptic_wrist::HapticWrist hw;

    haptic_wrist::jp_type pos;
    hw.gravity_compensate(true);
    hw.run();

    while (pm.getSafetyModule()->getMode() == barrett::SafetyModule::ACTIVE) {
        auto wamPose = wam.getToolPose();

        Eigen::Matrix4d tool_to_world = posQuatToTransform(boost::get<0>(wamPose), boost::get<1>(wamPose));
        hw.set_wrist_to_base(tool_to_world);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    hw.stop();

}
