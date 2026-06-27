# Simulink UDP LKAS Setup

The Python lane detector sends one UDP packet with four little-endian `single`
values:

```text
[lane_detected, offset_m, curvature_m, camera_status]
```

Each packet is 16 bytes.

## Simulink Blocks

1. Add a `UDP Receive` block.
2. Set the local port to `5005`.
3. Set the output data type to `uint8`.
4. Set the data size to `[16 1]`.
5. Add a `Byte Unpack` block.
6. Configure four outputs:
   - `single`
   - `single`
   - `single`
   - `single`
7. Set byte order to little-endian.
8. Connect the four outputs to a MATLAB Function block using `lkas_controller`.

## MATLAB Function Block Inputs

Use this input order:

```text
lane_detected
offset_m
curvature_m
camera_status
vehicle_speed
```

Use this output order:

```text
steer_cmd
lkas_active
lane_valid
```

## Python Example

If Python and MATLAB/Simulink run on the same computer:

```bash
python3 lane_detection_preview.py --udp-host 127.0.0.1 --udp-port 5005
```

If MATLAB/Simulink is on another computer, replace `127.0.0.1` with the MATLAB
PC IP address:

```bash
python3 lane_detection_preview.py --udp-host MATLAB_PC_IP --udp-port 5005
```
