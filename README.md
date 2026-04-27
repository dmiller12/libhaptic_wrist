
# Install Instructions
From project root
```bash
mkdir build && cd build
cmake ..
make
sudo make install
```
If you don't have the libbarrett dependency, you can disable related executables with:
```bash
cmake -DBUILD_BARRETT=OFF ..
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

On `MagnumOpus`, active joints map as:
- Wrist `ID1` -> WAM `J5`
- Wrist `ID2` -> WAM `J6`

The passive DoF is read from the MA600 on AUX2 of controller `ID1`.
Use `passive_encoder.offset_rad` and `passive_encoder.scale` in `haptic_wrist.yaml`
to map raw MA600 radians into physical passive-joint radians.
Control is joint-to-joint only (orientation target commands are disabled).

Make sure wam toolplate dh_params are correct. d should be 0.0

Zero position is set when haptic wrist is powered up, not when process started. Make sure the wrist is close to its zero position (within 1 motor revolution).

If present, reads config files from `~/.config/haptic_wrist`. Otherwise reads from `/etc/haptic_wrist`.
Overwrite the config dir location with env variable HAPTIC_WRIST_CONFIG_DIR

### Using pciefd

in haptic_wrist.yaml set transport_type: "pcie" and rebuild

then, either run ```source wam_ws/src/wam_teleop/scripts/can_init_pcifd.sh``` or manually setup the gripper as follows:

```
sudo modprobe peak_pciefd

sudo ip link set <your-can-interface> down || true
sudo ip link set <your-can-interface> type can bitrate 1000000 dbitrate 5000000 sjw 10 dsjw 5 sample-point 0.666 dsample-point 0.666 restart-ms 1000 fd on
sudo ip link set <your-can-interface> up
```

### Using usb

in haptic_wrist.yaml set transport_type: "usb" and rebuild

can interface does not need to be setup with usb like in pcie as we are using a serial connection.

**If the transport_type is not provided or an empty string is used, the default fdcanusb transport method will be used.**

### Common Moteus Commands
Zero the moteus
```bash
python3 -m moteus.moteus_tool --target 1,2 --zero-offset
```
Open tview:
```bash
python3 -m moteus_gui.tview --target 1,2
```
Open tview (wrist):
```bash
python3 -m moteus_gui.tview --devices=1,2 --fdcanusb /dev/serial/by-id/usb-mjbots_fdcanusb_C54FFEC3-if00
```
Open tview (gripper):
```bash
python3 -m moteus_gui.tview --devices=3 --fdcanusb /dev/serial/by-id/usb-mjbots_fdcanusb_5B75C352-if00
```
Find moteus serial devices:
```bash
ls -l /dev/serial/by-id/
```
