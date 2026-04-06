#include "haptic_wrist/haptic_wrist.h"
#include <barrett/units.h>
#include <iostream>
#include <string>

#include "tool_frame_cb.h"
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems.h>
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
template <size_t DOF>
int wam_main(int argc, char** argv, barrett::ProductManager& pm, barrett::systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    wam.gravityCompensate();

    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(true);
    hw.run();

    ToolFrameCb toolframeCb(pm.getExecutionManager(), &hw);
    barrett::systems::connect(wam.toolPose.output, toolframeCb.input);

    std::cout << "Press [Enter] to print current joint poses. Type q and press [Enter] to quit." << std::endl;
    std::string line;
    while (std::cout << ">>> " && std::getline(std::cin, line)) {
        if (line == "q" || line == "quit" || line == "exit") {
            break;
        }

        if (!line.empty() && line != "p" && line != "print") {
            std::cout << "Unknown command. Use [Enter]/p/print to capture a pose, or q to quit." << std::endl;
            continue;
        }

        auto wam_jp = wam.getJointPositions();
        auto wrist_jp = hw.getPosition();
        auto wrist_passive = hw.getPassivePosition();
        std::cout << "WAM joints (rad):\n" << wam_jp << std::endl;
        std::cout << "Wrist active joints [ID1, ID2] (rad):\n" << wrist_jp << std::endl;
        std::cout << "Wrist passive joint (rad): " << wrist_passive << std::endl;
    }

    pm.getSafetyModule()->waitForMode(barrett::SafetyModule::IDLE);

    hw.stop();
    return 0;
}
