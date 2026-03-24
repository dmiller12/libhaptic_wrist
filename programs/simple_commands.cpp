
#include "haptic_wrist/haptic_wrist.h"
#include <iostream>
#include <unistd.h>

int main(int argc, char** argv) {

    (void)argc;
    (void)argv;

    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(false);
    hw.run();

    sleep(1);
    // Keep the wrist compliant in this monitor utility.
    hw.hold(false);

    while (true) {
        std::cout << "active joints [ID1, ID2] rad: " << hw.getPosition().transpose() << std::endl;
        std::cout << "passive joint rad: " << hw.getPassivePosition() << std::endl;
        sleep(1);
    }

    hw.stop();
    return 0;
}
