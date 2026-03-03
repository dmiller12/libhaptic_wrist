#include "haptic_wrist/haptic_wrist.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

constexpr std::size_t kNumJoints = static_cast<std::size_t>(haptic_wrist::jp_type::RowsAtCompileTime);
static_assert(kNumJoints > 0, "Joint count must be known at compile time.");

constexpr double kDegToRad = M_PI / 180.0;

void printHelp(bool usingDegrees) {
    std::cout << "Commands:\n"
              << "  <joint> <angle>  - Set a single joint (1-" << kNumJoints << ") to the angle in "
              << (usingDegrees ? "degrees" : "radians") << ". Example: \"2 -45\"\n"
              << "  home             - Move to the configured home pose\n"
              << "  print            - Print current joint positions\n"
              << "  help             - Show this message\n"
              << "  q/quit/exit      - Stop the program" << std::endl;
}

bool parseJointTargetCommand(const std::string& line, bool inputInDegrees, std::size_t& jointIdx, double& angleRad) {
    std::istringstream iss(line);
    std::string jointToken;
    if (!(iss >> jointToken)) {
        return false;
    }

    // Accept "j1" or "1"
    if (jointToken.size() > 1 && (jointToken[0] == 'j' || jointToken[0] == 'J')) {
        jointToken = jointToken.substr(1);
    }

    try {
        int parsed = std::stoi(jointToken);
        if (parsed < 1 || parsed > static_cast<int>(kNumJoints)) {
            std::cout << "Joint index must be between 1 and " << kNumJoints << "." << std::endl;
            return false;
        }
        jointIdx = static_cast<std::size_t>(parsed - 1);
    } catch (const std::exception&) {
        std::cout << "Could not parse joint index from \"" << jointToken << "\"." << std::endl;
        return false;
    }

    double angleInput;
    if (!(iss >> angleInput)) {
        std::cout << "Please provide an angle after the joint index." << std::endl;
        return false;
    }

    angleRad = inputInDegrees ? angleInput * kDegToRad : angleInput;
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
            hw.jointMoveTo(hw.getHome());
            hw.hold(true);
            continue;
        }

        if (line == "print") {
            auto current = hw.getPosition();
            std::cout << "Current joints (rad): " << current.transpose() << std::endl;
            continue;
        }

        std::size_t jointIdx = 0;
        double angleRad = 0.0;
        if (!parseJointTargetCommand(line, inputInDegrees, jointIdx, angleRad)) {
            std::cout << "Try again or type 'help'." << std::endl;
            continue;
        }

        haptic_wrist::jp_type target = hw.getPosition();
        target[jointIdx] = angleRad;

        std::cout << "Moving joint " << (jointIdx + 1) << " to "
                  << (inputInDegrees ? angleRad / kDegToRad : angleRad) << (inputInDegrees ? " deg" : " rad")
                  << std::endl;

        hw.jointMoveTo(target);
        hw.hold(true); // keep stiffness once motion completes
    }

    hw.stop();
    std::cout << "Exiting joint move console." << std::endl;
    return 0;
}
