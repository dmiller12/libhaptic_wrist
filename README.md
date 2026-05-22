
# Install Instructions
Assuming you are using [wam-ros-docker](https://github.com/ualberta-robotics/wam-ros-docker).

This branch is for the end effector wrist ![EE wrist](https://github.com/sergey-khl/wam_teleop/blob/EEWrist/media/ee_wrist.jpg).
```bash
cd /home/user/wam_ros
source build_haptic.sh
```
To run tests, from the build directory execute:
```bash
ctest
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

### Configuring a PEAK CAN FD PCIe card
see [wam_teleop](https://github.com/sergey-khl/wam_teleop/blob/EEWrist/README.md) and [can_init_pcifd](https://github.com/sergey-khl/wam_teleop/blob/EEWrist/scripts/can_init_pcifd.sh) for examples.
We tend to reserve can0 and can1 for the WAM and can2 and can3 for the moteus controlled wrists.

Finally, update the config and ensure `transport_type` matches the interface name you want to use (usb or pcie)

If the transport_args are not provided or an empty string is used, the default fdcanusb transport method will be used.

### Common Moteus Commands
Zero the moteus
```bash
python3 -m moteus.moteus_tool --target 1,2,3 --zero-offset
```
Open tview:
```bash
python3 -m moteus_gui.tview --target 1,2,3
```

You may need to add additional arguments if the above does not work. For example,
```bash
python3 -m moteus_gui.tview --devices=1,2,3 --fdcanusb /dev/serial/by-id/usb-mjbots_fdcanusb_5B75C352-if00
```
if using usb. Or, if using PCI:
```bash
python3 -m moteus_gui.tview --can-iface socketcan --device 3 --can-chan can2
```
Note that the details of device #, can# and /dev/serial/by-id may be different for you.
