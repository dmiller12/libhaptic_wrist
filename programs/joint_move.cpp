#include "haptic_wrist/haptic_wrist.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kNumJoints = haptic_wrist::kWristDofs;

void printHelp(bool usingDegrees) {
    std::cout << "Enter " << kNumJoints
              << " joint angles in " << (usingDegrees ? "degrees" : "radians")
              << " separated by spaces (e.g. \"0 -90 10 0\").\n"
                 "Type 'home' to move to the configured home position, or 'q' to quit."
              << std::endl;
}

bool parseJointTargets(const std::string& line, bool inputInDegrees, haptic_wrist::jp_type& out) {
    constexpr double kDegToRad = M_PI / 180.0;
    std::istringstream iss(line);
    std::vector<double> values;
    double value;
    while (iss >> value) {
        values.push_back(value);
    }

    if (values.size() != kNumJoints) {
        std::cout << "Expected " << kNumJoints << " values but received " << values.size() << "." << std::endl;
        return false;
    }

    for (std::size_t i = 0; i < kNumJoints; ++i) {
        out[i] = inputInDegrees ? values[i] * kDegToRad : values[i];
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    bool inputInDegrees = true;
    if (argc >= 2) {
        std::string arg(argv[1]);
        if (arg == "--rad" || arg == "--radians") {
            inputInDegrees = false;
        } else if (arg == "--deg" || arg == "--degrees") {
            inputInDegrees = true;
        } else {
            std::cout << "Unknown option '" << arg << "'. Use --deg (default) or --rad." << std::endl;
        }
    }

    haptic_wrist::HapticWrist hw;
    hw.gravityCompensate(false);
    hw.run();

    std::cout << "Simple joint move console (" << (inputInDegrees ? "degrees" : "radians") << ")\n";
    printHelp(inputInDegrees);

    std::string line;
    bool running = true;

    while (running && std::cout << "\n>>> " && std::getline(std::cin, line)) {
        if (line.empty()) {
            continue;
        }

        if (line == "q" || line == "quit" || line == "exit") {
            running = false;
            break;
        }

        if (line == "help") {
            printHelp(inputInDegrees);
            continue;
        }

        if (line == "home") {
            std::cout << "Moving to home position..." << std::endl;
            hw.moveTo(hw.getHome());
            hw.hold(true);
            continue;
        }

        haptic_wrist::jp_type target = haptic_wrist::jp_type::Zero();
        if (!parseJointTargets(line, inputInDegrees, target)) {
            std::cout << "Try again or type 'help'." << std::endl;
            continue;
        }

        std::cout << "Moving to: ";
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            std::cout << std::fixed << std::setprecision(4) << target[i] << (i + 1 == kNumJoints ? "" : ", ");
        }
        std::cout << std::endl;

        hw.moveTo(target);
        hw.hold(true); // keep stiffness once motion completes
    }

    hw.stop();
    std::cout << "Exiting joint move console." << std::endl;
    return 0;
}
