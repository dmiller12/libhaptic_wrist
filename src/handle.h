#pragma once

#include "haptic_wrist/types.h"
#include <vector>
#include <boost/optional.hpp>

namespace haptic_wrist {

class Handle {
  public:
    Handle(const char* device_path = "/dev/input/js0");
    ~Handle();

    bool is_connected() const;

    // Read pending events from the joystick
    void poll();

    // Returns the current parsed state (bumper, trigger), or boost::none if not connected
    boost::optional<handle_type> get_state() const;

  private:
    int joy_fd_ = -1;
    std::vector<int> joy_axes_;
    std::vector<int> joy_buttons_;
};

} // namespace haptic_wrist
