#include "haptic_wrist/haptic_wrist.h"
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <boost/optional.hpp>
#include <thread>
#include <chrono>


int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(false);
    hw.run();

    while (true) {
        if (boost::optional<haptic_wrist::handle_type> opt_handle = hw.getHandle()) {
            haptic_wrist::handle_type handle = *opt_handle; 

            std::cout << "joyx: " << handle[0] 
                    << " | joyy: " << handle[1] 
                    << " | bumper: " << handle[2] 
                    << " | trigger: " << handle[3] << std::endl;
        }
        sleep(0.1);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    hw.stop();
    return 0;
}
