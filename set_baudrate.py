import time

import serial

PORT = "COM7"


def change_sensor_baudrate(port: str, current_baud: int, target_baud: int):
    # code to register 0x04 based on WTVB01-485's manual 10.4.2
    baud_codes = {
        4800: "50 06 00 04 00 01 04 4A",
        9600: "50 06 00 04 00 02 44 4B",
        19200: "50 06 00 04 00 03 85 8B",
        38400: "50 06 00 04 00 04 C4 49",
        57600: "50 06 00 04 00 05 05 89",
        115200: "50 06 00 04 00 06 45 88",
        230400: "50 06 00 04 00 07 84 48",
    }

    if target_baud not in baud_codes:
        raise ValueError(f"Baud rate {target_baud} is not supported")
    print(f"Connecting to {port} at {current_baud} bps...")
    with serial.Serial(port, current_baud, timeout=0.5) as ser:
        # unlock register (valid for 10 seconds)
        ser.write(bytes.fromhex("50 06 00 69 B5 88 22 A1"))
        time.sleep(0.1)

        # send command to sensor to change register 0x04
        ser.write(bytes.fromhex(baud_codes[target_baud]))
        time.sleep(0.1)

    print(f"Sensor is now permanently in {target_baud} bps.")


if __name__ == "__main__":
    # switch from 9600 to 115200
    change_sensor_baudrate(PORT, 9600, 115200)

    # switch from 115200 to 9600 bps
    # change_sensor_baudrate(PORT, 115200, 9600)
