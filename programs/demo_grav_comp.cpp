// #include "haptic_wrist/haptic_wrist.h"
// #include <barrett/units.h>

// #include <barrett/products/product_manager.h>
// #include <barrett/standard_main_function.h>
// #include <barrett/systems.h>
// #include "tool_frame_cb.h"

// BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;


// template <size_t DOF>
// int wam_main(int argc, char **argv, barrett::ProductManager &pm, barrett::systems::Wam<DOF> &wam) {
//     BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
//     wam.gravityCompensate();

//     haptic_wrist::HapticWrist hw;
//     hw.gravityCompensate(true);
//     hw.run();

//     ToolFrameCb toolframeCb(pm.getExecutionManager(), &hw);
//     barrett::systems::connect(wam.toolPose.output, toolframeCb.input);

//     while (true) {
//         std::cout << "position\n" << hw.getPosition() << std::endl;
//         sleep(1);
//     }

//     pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);

//     hw.stop();
//     return 0;
// }
