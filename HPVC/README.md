# HPVC / Raspberry Pi #1

This directory contains the HPVC-side source and operating scripts for the RC car
zonal architecture setup.

## Current Network

```text
PC / HMI / OTA
  <-> Wi-Fi
Raspberry Pi #1 / HPVC
  <-> vehicle Ethernet switch
Raspberry Pi #2 / Center Zone
TC375 Front Zone
TC375 Rear Zone
```

Current lab IPs:

```text
RPi #1 Wi-Fi        192.168.219.104
RPi #2 Wi-Fi        192.168.219.105
RPi #1 eth0         192.168.10.1
TC375 Front         192.168.10.11
TC375 Front MAC     00:00:0c:11:11:11
```

## UDP Ports

```text
RPi #2 -> RPi #1 lane/side ultrasonic : UDP 5005
RPi #1 -> TC375 Front steering command: UDP 5100, source 5101
TC375 Front -> RPi #1 steering status : UDP 5102
TC375 Front -> RPi #1 front sensors   : UDP 5011
```

## Files

```text
RPI1/                     MATLAB/Python HPVC-side source files
Interfaces/               UDP protocol notes
scripts/start_rpi1.sh     RPi #1 runtime startup script
```

Generated binaries, object files, logs, `slprj`, and `*_ert_rtw` directories are
intentionally not included in this repository.

## Start RPi #1

On Raspberry Pi #1:

```sh
chmod +x ~/start_rpi1.sh
~/start_rpi1.sh
```

The script configures `eth0` as `192.168.10.1/24`, installs the static TC375
front ARP entry, and starts `RPI1Deployment.elf`.

## Useful Checks

Check RPi #2 packets:

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
