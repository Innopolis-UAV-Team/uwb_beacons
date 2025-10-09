# DW-1000 RTK

## Introduction

This is a simple RTK implementation for the DW-1000 UWB radio. It is based on the [DW1000 library](https://usermanual.wiki/Pdf/DW1000SoftwareAPIGuiderev2p4.1120642274.pdf) and [DW1000 user guide](https://www.sunnywale.com/uploadfile/2021/1230/DW1000%20User%20Manual_Awin.pdf).

## Usage

The RTK utilizes two UWB devices type: a base station (router) and a tag (anchor).
To build the code for the router, select the anchor ID (default is 1) and run the following command:

```bash
make ID=1 router-upload
```

To build the code for the tag, select the anchor ID (default is 1) and run the following command:

```bash
make ID=2 anchor-upload
```

The base station will listen for the tag and send the RTK solution via UART.

### Parce UART output

The UART is used to send the distance to all found tags. The distance is sent in cm.

The format is:

```
| uint8_t | uint32_t | uint32_t |
| ------- | -------- | -------- |
| TAG ID  | DISTANCE | 0xFFFFFF00 - message end |
```

Example:

```
real distance: 100 mm
1,100,0xFF00
in hex:
0x1, 0x00, 0x00 0x00 0x64, 0xFF, 0xFF, 0xFF, 0x00
```

# Data parsing


Use the script to check if UART is working.

```bash
python3 check.py --port /dev/ttyUSB0 --baudrate 460800
```

There is a python script that parses the data and saves it to a file.

```bash
python3 save_test_data.py --port /dev/ttyUSB0 --baudrate 460800 --real_distance 100
```

The script will save the data to a file in the `test/data` directory.

Get statistics for the data:
```bash
python3 get_stats.py --input_file test/data/*.txt --output test/data/stats/ --real_values_file test/data/real_values.*
```

The script will save the statistics to a file in the `test/data/stats` directory and separate all anchors data to `id.csv` files, if the `--real_values_file` argument is provided, the real values will be saved to the file.
The format of the statistics file is:

| id | count | mean | min | max | std | median | real_value | error | error_pct |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 2442 | 536.3611793611793 | 31.0 | 635.0 | 23.25916398894732 | 536.5 | 500.0 | 36.361179361179325 | 7.272235872235864 |

# Data parsing

To parse the messages use Message class from the [common folder](test/common/serial_messages.py)

> [!NOTE]
> The baudrate which is set by user in the software will be doubled because of some error in the hardware/clock configuration.
