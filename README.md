# DW-1000 RTK

## Introduction

This is a simple RTK implementation for the DW-1000 UWB radio. It is based on the [DW1000 library](https://usermanual.wiki/Pdf/DW1000SoftwareAPIGuiderev2p4.1120642274.pdf).

## Usage

The RTK utilizes two UWB devices type: a base station (router) and a tag (anchor).
To build the code for the router, run the following command:

```bash
make router
```

To build the code for the tag, select the anchor ID (default is 1) and run the following command:

```bash
make anchor ID=1
```

The base station will listen for the tag and send the RTK solution via UART.

### Parce UART output

The UART is used to send the distance to all found tags. The distance is sent in cm.

The format is:

```
| uint8_t | uint16_t | uint16_t |
| ------- | -------- | -------- |
| TAG ID  | DISTANCE | 0xFF00 - message end |
```

Example:

```
real distance: 100 cm
1,100,0xFF00
in hex:
0x1, 0x00, 0x64, 0xFF, 0x00
```

# Data parsing


Use the script to check if UART is working.

```bash
python3 check.py --port /dev/ttyUSB0 --baudrate 230400
```

There is a python script that parses the data and saves it to a file.

```bash
python3 save_test_data.py --port /dev/ttyUSB0 --baudrate 230400 --real_distance 100
```

The script will save the data to a file in the `test/data` directory.
