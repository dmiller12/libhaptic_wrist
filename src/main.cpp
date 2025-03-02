

#include "haptic_wrist/haptic_wrist.h"

int main(int arc, char** argv) {

    haptic_wrist::HapticWrist hw;
    
    haptic_wrist::jp_type pos;
    pos << 0.0, 0.0, 0.0;
    hw.set_position(pos);
    hw.run();

    sleep(1);
    pos << 0.5, 0.0, 0.0;
    hw.move_to(pos);

    sleep(1);
    pos << -0.5, 0.0, 0.0;
    hw.move_to(pos);

    sleep(1);
    pos << 0.0, 0.0, 0.0;
    hw.move_to(pos);

    sleep(1);
    pos << 0.0, 0.5, 0.0;
    hw.move_to(pos);

    sleep(1);
    pos << 0.0, -0.5, 0.0;
    hw.move_to(pos);

    sleep(1);
    pos << -0.5, -0.5, 0.0;
    hw.move_to(pos);

    while (true) {
        std::cout << hw.get_position() << std::endl;
        sleep(1);
    }

    hw.stop();

    return 0;
}
