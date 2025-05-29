#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <signal.h>

#include "haptic_wrist/haptic_wrist.h"

static bool running = true;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    running = false;
}

void printStatus(haptic_wrist::HapticWrist& wrist) {
    auto pos = wrist.getPosition();
    auto vel = wrist.getVelocity();
    auto torque = wrist.getTorque();
    
    std::cout << "Position [rad]: [" 
              << std::fixed << std::setprecision(3)
              << pos(0) << ", " << pos(1) << ", " << pos(2) << "]" << std::endl;
    std::cout << "Velocity [rad/s]: [" 
              << vel(0) << ", " << vel(1) << ", " << vel(2) << "]" << std::endl;
    std::cout << "Torque [N⋅m]: [" 
              << torque(0) << ", " << torque(1) << ", " << torque(2) << "]" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
}

int main() {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "Simple Hold Command Test" << std::endl;
    std::cout << "=======================" << std::endl;
    
    try {
        // Initialize the wrist
        haptic_wrist::HapticWrist hw;
        
        // Disable gravity compensation
        std::cout << "Disabling gravity compensation..." << std::endl;
        hw.gravityCompensate(false);
        
        // Start the control loop
        std::cout << "Starting wrist control loop..." << std::endl;
        hw.run();
        
        // Wait a moment for initialization
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\nInitial status:" << std::endl;
        printStatus(hw);
        
        // Enable hold at current position
        // std::cout << "\nEnabling HOLD at current position..." << std::endl;
        // hw.hold(true);
        
        // // Monitor while holding for 5 seconds
        // std::cout << "Holding position for 5 seconds (try to move the wrist manually)..." << std::endl;
        // for (int i = 1; i <= 500000 && running; i++) {
        //     std::cout << "\nHold time: " << i << "s" << std::endl;
        //     printStatus(hw);
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        // }

        hw.moveTo({0, 0, 0});
        // sleep(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        
        
        std::cout << "\nStopping wrist control..." << std::endl;
        hw.stop();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Hold test finished." << std::endl;
    return 0;
}