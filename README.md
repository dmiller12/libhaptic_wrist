
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
Control is joint-to-joint only (orientation target commands are disabled).

Make sure wam toolplate dh_params are correct. d should be 0.0

Zero position is set when haptic wrist is powered up, not when process started. Make sure the wrist is close to its zero position (within 1 motor revolution).

If present, reads config files from `~/.config/haptic_wrist`. Otherwise reads from `/etc/haptic_wrist`.
Overwrite the config dir location with env variable HAPTIC_WRIST_CONFIG_DIR

### Configuring a PEAK CAN FD PCIe card
If using the peak CANFD PCIe card, first identify the network interface name assigned to the CANFD card:

The interface can be found by loading the module:

```bash
sudo modprobe peak_pciefd
```
and running the following command:

```bash
dmesg | grep peak_pciefd
```
Now configure the interface. Replace your `<your-can-interface` with your actual interface name.
```bash
sudo modprobe peak_pciefd

ip link set <your-can-interface> up type can \
  bitrate 1000000 dbitrate 5000000 \
  sjw 10 dsjw 5 \
  sample-point 0.666 dsample-point 0.666 \
  restart-ms 1000 fd on
```
Finally, update the config and ensure `transport_args` matches the interface name you identified and configured.
```yaml
moteus:
  # ... other settings
  transport_args: ["--socketcan-iface", "<your-can-interface>"]
```

**If the transport_args are not provided or an empty string is used, the default fdcanusb transport method will be used.**

### Common Moteus Commands
Zero the moteus
```bash
python3 -m moteus.moteus_tool --target 1,2 --zero-offset
```
Open tview:
```bash
python3 -m moteus_gui.tview --target 1,2
```
