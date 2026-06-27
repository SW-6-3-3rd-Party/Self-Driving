# HPVC

This directory contains the HPVC-side source and operating scripts for the RC car
zonal architecture setup.

## Current Network

```text
PC / HMI / OTA
  <-> Wi-Fi
HPVC / Vehicle Controller
  <-> vehicle Ethernet switch
MIDDLE / Center Zone
TC375 Front Zone
TC375 Rear Zone
```

Current lab IPs:

```text
HPVC Wi-Fi        192.168.219.104
MIDDLE Wi-Fi        192.168.219.105
HPVC eth0         192.168.10.1
TC375 Front         192.168.10.11
TC375 Front MAC     02:37:50:ae:b0:01
```

## UDP Ports

```text
MIDDLE -> HPVC lane/side/object     : UDP 5005
HPVC -> TC375 Front steering command: UDP 5100, source 5101
TC375 Front -> HPVC steering status : UDP 5102
TC375 Front -> HPVC front sensors   : UDP 5011
HPVC -> brake controller request    : UDP 5013, optional
```

## Files

```text
Models/                   Simulink validation/deployment models
Interfaces/               UDP protocol notes
scripts/start_hpvc.sh     HPVC runtime startup script
hpvc_aeb.py               HPVC-side AEB fusion runtime
```

## HPVC AEB Runtime

For bench validation without rebuilding the Simulink model, HPVC can run the
Python HPVC AEB receiver. It fuses MIDDLE person detections with Front
TC375 ToF/ultrasonic distances and prints the AEB state. Add `--brake-host` to
transmit the `HPAB` brake request packet to a downstream brake controller.

```sh
python3 -m HPVC.hpvc_aeb
python3 -m HPVC.hpvc_aeb --brake-host 192.168.10.12 --brake-port 5013
```

Generated binaries, object files, logs, `slprj`, and `*_ert_rtw` directories are
intentionally not included in this repository.

## Start HPVC

On HPVC:

```sh
chmod +x ~/start_hpvc.sh
~/start_hpvc.sh
```

The script configures `eth0` as `192.168.10.1/24`, installs the static TC375
front ARP entry, and starts `HPVCDeployment.elf`.

The deployment model is generated with `LKAS Enable=true` so HPVC sends real
`SteeringValid` HPSC commands when the MIDDLE link/lane path is valid. If the
input becomes invalid, HPVC sends `EmergencyCenter` instead.

## Useful Checks

Check MIDDLE packets:

```sh
sudo tcpdump -ni wlan0 udp and src host 192.168.219.105 and dst port 5005
```

Check TC375 front traffic:

```sh
sudo tcpdump -ni eth0 "udp and host 192.168.10.11 and (port 5100 or port 5102 or port 5011)"
```

Expected TC375 front traffic:

```text
192.168.10.1.5101  > 192.168.10.11.5100  steering command
192.168.10.11.x    > 192.168.10.1.5011   front sensor data
192.168.10.11.x    > 192.168.10.1.5102   steering status
```
