#include "handle.h"
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

// this is for the sony navigation controller

namespace haptic_wrist {

Handle::Handle(const char* device_path) {
    joy_fd_ = open(device_path, O_RDONLY | O_NONBLOCK);
    if (joy_fd_ >= 0) {
        joy_axes_.resize(10, 0); 
        joy_buttons_.resize(15, 0);
    }
}

Handle::~Handle() {
    if (joy_fd_ >= 0) {
        close(joy_fd_);
    }
}

bool Handle::is_connected() const {
    return joy_fd_ >= 0;
}

void Handle::poll() {
    if (joy_fd_ < 0) return;

    struct js_event event;
    while (read(joy_fd_, &event, sizeof(event)) > 0) {
        event.type &= ~JS_EVENT_INIT;

        if (event.type == JS_EVENT_AXIS) {
            if (event.number < joy_axes_.size()) {
                joy_axes_[event.number] = event.value; 
            }
        } else if (event.type == JS_EVENT_BUTTON) {
            if (event.number < joy_buttons_.size()) {
                joy_buttons_[event.number] = event.value; 
            }
        }
    }
}

boost::optional<handle_type> Handle::get_state() const {
    if (joy_fd_ < 0) return boost::none;

    handle_type current_joy;
    // Bumper
    current_joy(0) = joy_buttons_[4]; 
    // Trigger
    current_joy(1) = joy_buttons_[5];   

    return current_joy;
}

} // namespace haptic_wrist
