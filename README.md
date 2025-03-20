
# Install Instructions
From project root
```bash
mkdir build && cd build
cmake ..
make
sudo make install
```

## Integrate with your project
Add to CMakeLists.txt
```bash
find_package(haptip_wrist REQUIRED)
...
target_link_libraries(your_target haptic_wrist) 
```
See `programs/demo_grav_comp.cpp` and `tool_frame_cb.h` for integrating with the wam.
## Notes

The joint numbers match the wam wrist, that is J1, J2, J3 correspond with J5, J6, J7 on the wam wrist, respectively.

Make sure wam toolplate dh_params are correct. d should be 0.0

Zero position is set when haptic wrist is powered up, not when process started. Make sure the wrist is close to its zero position (within 1 motor revolution).

If present, reads config files from `~/.config/haptic_wrist`. Otherwise reads from `/etc/haptic_wrist`.
Overwrite the config dir location with env variable HAPTIC_WRIST_CONFIG_DIR
