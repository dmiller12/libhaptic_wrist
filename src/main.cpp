

#include "haptic_wrist/haptic_wrist.h"

int main(int arc, char** argv) {

    haptic_wrist::HapticWrist hw;
    
    haptic_wrist::jp_type pos;
    // pos << 0.0, 0.0, 0.0;
    // hw.set_position(pos);
    hw.gravity_compensate(true);
    hw.run();

    // sleep(1);
    // pos << 0.5, 0.0, 0.0;
    // hw.move_to(pos);
    //
    // sleep(1);
    // pos << -0.5, 0.0, 0.0;
    // hw.move_to(pos);
    //
    // sleep(1);
    // pos << 0.0, 0.0, 0.0;
    // hw.move_to(pos);
    //
    // sleep(1);
    // pos << 0.0, 0.5, 0.0;
    // hw.move_to(pos);
    //
    // sleep(1);
    // pos << 0.0, -0.5, 0.0;
    // hw.move_to(pos);
    //
    // sleep(1);
    // pos << -0.5, -0.5, 0.0;
    // hw.move_to(pos);

    // sleep(10);
    //
    // sleep(1);
    // pos << 0, 0, 0.5;
    // hw.move_to(pos);
    // std::cout << hw.get_position() << std::endl;
    //
    // sleep(1);
    // pos << 0, 0, -0.5;
    // hw.move_to(pos);
    // std::cout << hw.get_position() << std::endl;

    // while (true) {
    //     std::cout << hw.get_position() << std::endl;
    //     sleep(1);
    // }
    sleep(10);

    std::cout << "acutal torque\n" <<  hw.get_torque() << std::endl;

    hw.stop();

    return 0;
}
