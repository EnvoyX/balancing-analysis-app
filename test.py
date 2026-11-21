import time

import device_model

"""
    WTVB01-485 Example
"""

# region Common Register Address Reference Table
"""

hex     dec     description

0x00    0       Save / Reboot / Factory Reset
0x04    4       Serial Baud Rate

0x1A    26      Device Modbus Address

0x34    52      X-axis acceleration g
0x35    53      Y-axis acceleration g
0x36    54      Z-axis acceleration g

0x3A    58      Vibration Velocity X
0x3B    59      Vibration Velocity Y
0x3C    60      Vibration Velocity Z

0x3D    61      Vibration Angle X
0x3E    62      Vibration Angle Y
0x3F    63      Vibration Angle Z

0x40    64      Temperature

0x41    65      Vibration Displacement X
0x42    66      Vibration Displacement Y
0x43    67      Vibration Displacement Z

0x44    68      Vibration Frequency X
0x45    69      Vibration Frequency Y
0x46    70      Vibration Frequency Z

0x63    99      Cutoff Frequency
0x64    100     Cutoff Frequency
0x65    101     Detection Cycle

"""
# endregion

# Instantiate the device model
# changed baud from 9600 to 115200  so that python able to collect at minimum 100 sample per second
# change the baudrate in set_baudrate.py
device = device_model.DeviceModel("Laptop Device", "COM7", 115200, 0x50)
# Open the device
device.openDevice()
# Start loop polling
device.startLoopRead()
time.sleep(0.5)

# Display sensor data
while True:
    # v: vibration velocity | a: vibration angle | t: temperature | s: vibration displacement | f: vibration frequency
    # print(
    #     "vx:{} vy:{} vz:{} ax:{} ay:{} az:{} t:{} sx:{} sy:{} sz:{} fx:{} fy:{} fz:{}".format(
    #         device.get("58"),
    #         device.get("59"),
    #         device.get("60"),
    #         device.get("61"),
    #         device.get("62"),
    #         device.get("63"),
    #         device.get("64"),
    #         device.get("65"),
    #         device.get("66"),
    #         device.get("67"),
    #         device.get("68"),
    #         device.get("69"),
    #         device.get("70"),
    #     )
    # )
    # AX: Acceleration X | AY: Acceleration Y | AZ: Acceleration Z
    print(
        "AX:{} AY:{} AZ:{}".format(
            device.get("52"),
            device.get("53"),
            device.get("54"),
        )
    )
    time.sleep(0.2)

# Read register: read 1 register starting from 0x3A
# device.readReg(0x3a, 1)
# Get the read result
# device.get(str(0x3a))

# Write register: write 50 to 0x65 (sets the detection cycle to 50 Hz)
# device.writeReg(0x65, 50)
