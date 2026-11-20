# coding:UTF-8
import threading
import time

import serial
from serial import SerialException


# Serial port configuration
class SerialConfig:
    # Serial port name
    portName = ""

    # Baud rate
    baud = 9600


# Device instance
class DeviceModel:
    # region Attributes

    # Device name
    deviceName = "My Device"

    # Device Modbus ID
    ADDR = 0x50

    # Device data dictionary
    deviceData = {}

    # Whether the device is open
    isOpen = False

    # Whether to read in a loop
    loop = False

    # Serial port
    serialPort = None

    # Serial port configuration
    serialConfig = SerialConfig()

    # Temporary byte buffer
    TempBytes = []

    # Starting register
    statReg = None

    # endregion

    # region CRC Calculation
    auchCRCHi = [
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x01,
        0xC0,
        0x80,
        0x41,
        0x00,
        0xC1,
        0x81,
        0x40,
    ]

    auchCRCLo = [
        0x00,
        0xC0,
        0xC1,
        0x01,
        0xC3,
        0x03,
        0x02,
        0xC2,
        0xC6,
        0x06,
        0x07,
        0xC7,
        0x05,
        0xC5,
        0xC4,
        0x04,
        0xCC,
        0x0C,
        0x0D,
        0xCD,
        0x0F,
        0xCF,
        0xCE,
        0x0E,
        0x0A,
        0xCA,
        0xCB,
        0x0B,
        0xC9,
        0x09,
        0x08,
        0xC8,
        0xD8,
        0x18,
        0x19,
        0xD9,
        0x1B,
        0xDB,
        0xDA,
        0x1A,
        0x1E,
        0xDE,
        0xDF,
        0x1F,
        0xDD,
        0x1D,
        0x1C,
        0xDC,
        0x14,
        0xD4,
        0xD5,
        0x15,
        0xD7,
        0x17,
        0x16,
        0xD6,
        0xD2,
        0x12,
        0x13,
        0xD3,
        0x11,
        0xD1,
        0xD0,
        0x10,
        0xF0,
        0x30,
        0x31,
        0xF1,
        0x33,
        0xF3,
        0xF2,
        0x32,
        0x36,
        0xF6,
        0xF7,
        0x37,
        0xF5,
        0x35,
        0x34,
        0xF4,
        0x3C,
        0xFC,
        0xFD,
        0x3D,
        0xFF,
        0x3F,
        0x3E,
        0xFE,
        0xFA,
        0x3A,
        0x3B,
        0xFB,
        0x39,
        0xF9,
        0xF8,
        0x38,
        0x28,
        0xE8,
        0xE9,
        0x29,
        0xEB,
        0x2B,
        0x2A,
        0xEA,
        0xEE,
        0x2E,
        0x2F,
        0xEF,
        0x2D,
        0xED,
        0xEC,
        0x2C,
        0xE4,
        0x24,
        0x25,
        0xE5,
        0x27,
        0xE7,
        0xE6,
        0x26,
        0x22,
        0xE2,
        0xE3,
        0x23,
        0xE1,
        0x21,
        0x20,
        0xE0,
        0xA0,
        0x60,
        0x61,
        0xA1,
        0x63,
        0xA3,
        0xA2,
        0x62,
        0x66,
        0xA6,
        0xA7,
        0x67,
        0xA5,
        0x65,
        0x64,
        0xA4,
        0x6C,
        0xAC,
        0xAD,
        0x6D,
        0xAF,
        0x6F,
        0x6E,
        0xAE,
        0xAA,
        0x6A,
        0x6B,
        0xAB,
        0x69,
        0xA9,
        0xA8,
        0x68,
        0x78,
        0xB8,
        0xB9,
        0x79,
        0xBB,
        0x7B,
        0x7A,
        0xBA,
        0xBE,
        0x7E,
        0x7F,
        0xBF,
        0x7D,
        0xBD,
        0xBC,
        0x7C,
        0xB4,
        0x74,
        0x75,
        0xB5,
        0x77,
        0xB7,
        0xB6,
        0x76,
        0x72,
        0xB2,
        0xB3,
        0x73,
        0xB1,
        0x71,
        0x70,
        0xB0,
        0x50,
        0x90,
        0x91,
        0x51,
        0x93,
        0x53,
        0x52,
        0x92,
        0x96,
        0x56,
        0x57,
        0x97,
        0x55,
        0x95,
        0x94,
        0x54,
        0x9C,
        0x5C,
        0x5D,
        0x9D,
        0x5F,
        0x9F,
        0x9E,
        0x5E,
        0x5A,
        0x9A,
        0x9B,
        0x5B,
        0x99,
        0x59,
        0x58,
        0x98,
        0x88,
        0x48,
        0x49,
        0x89,
        0x4B,
        0x8B,
        0x8A,
        0x4A,
        0x4E,
        0x8E,
        0x8F,
        0x4F,
        0x8D,
        0x4D,
        0x4C,
        0x8C,
        0x44,
        0x84,
        0x85,
        0x45,
        0x87,
        0x47,
        0x46,
        0x86,
        0x82,
        0x42,
        0x43,
        0x83,
        0x41,
        0x81,
        0x80,
        0x40,
    ]

    # endregion CRC Calculation

    def __init__(self, deviceName, portName, baud, ADDR):
        print("Initializing device model")
        # Device name (custom)
        self.deviceName = deviceName
        # Serial port name
        self.serialConfig.portName = portName
        # Serial baud rate
        self.serialConfig.baud = baud
        # Modbus device address
        self.ADDR = ADDR

    # Calculate CRC checksum
    def get_crc(self, datas, dlen):
        tempH = 0xFF  # High CRC byte initialization
        tempL = 0xFF  # Low CRC byte initialization
        for i in range(0, dlen):
            tempIndex = (tempH ^ datas[i]) & 0xFF
            tempH = (tempL ^ self.auchCRCHi[tempIndex]) & 0xFF
            tempL = self.auchCRCLo[tempIndex]
        return (tempH << 8) | tempL
        pass

    # region Get/Set Device Data

    # Set device data
    def set(self, key, value):
        # Store device data under the given key
        self.deviceData[key] = value

    # Get device data
    def get(self, key):
        # Retrieve data by key; return None if not found
        if key in self.deviceData:
            return self.deviceData[key]
        else:
            return None

    # Delete device data
    def remove(self, key):
        # Delete device key-value entry
        del self.deviceData[key]

    # endregion

    # Open device
    def openDevice(self):
        # Close the port first
        self.closeDevice()
        try:
            self.serialPort = serial.Serial(
                self.serialConfig.portName, self.serialConfig.baud, timeout=0.5
            )
            self.isOpen = True
            print("{} is open".format(self.serialConfig.portName))
            # Start a thread to continuously listen for serial data
            t = threading.Thread(
                target=self.readDataTh,
                args=(
                    "Data-Received-Thread",
                    10,
                ),
            )
            t.start()
            print("Device opened successfully")
        except SerialException:
            print("Failed to open " + self.serialConfig.portName)

    # Serial port listener thread
    def readDataTh(self, threadName, delay):
        print("Starting " + threadName)
        while True:
            # If the serial port is open
            if self.isOpen:
                try:
                    tLen = self.serialPort.inWaiting()
                    if tLen > 0:
                        data = self.serialPort.read(tLen)
                        self.onDataReceived(data)
                except Exception as ex:
                    print(ex)
            else:
                time.sleep(0.1)
                print("Serial port is not open")
                break

    # Close device
    def closeDevice(self):
        if self.serialPort is not None:
            self.serialPort.close()
            print("Port closed")
        self.isOpen = False
        print("Device closed")

    # region Data Parsing

    # Serial port data handler
    def onDataReceived(self, data):
        tempdata = bytes.fromhex(data.hex())
        for val in tempdata:
            self.TempBytes.append(val)
            # Check if device ID matches
            if self.TempBytes[0] != self.ADDR:
                del self.TempBytes[0]
                continue
            # Check if function code is 03 (Read Holding Registers)
            if len(self.TempBytes) > 2:
                if not (self.TempBytes[1] == 0x03):
                    del self.TempBytes[0]
                    continue
                tLen = len(self.TempBytes)
                # Full protocol packet received
                if tLen == self.TempBytes[2] + 5:
                    # CRC verification
                    tempCrc = self.get_crc(self.TempBytes, tLen - 2)
                    if (tempCrc >> 8) == self.TempBytes[tLen - 2] and (
                        tempCrc & 0xFF
                    ) == self.TempBytes[tLen - 1]:
                        self.processData(self.TempBytes[2])
                    else:
                        del self.TempBytes[0]

    # Parse register values
    def processData(self, length):
        # Retrieve starting register from the read command
        if self.statReg is not None:
            for i in range(int(length / 2)):
                # Combine high and low bytes into 16-bit register value
                value = self.TempBytes[2 * i + 3] << 8 | self.TempBytes[2 * i + 4]
                # Parse vibration angle
                if 0x3D <= self.statReg <= 0x3F:
                    value = value / 32768 * 180
                    self.set(str(self.statReg), value)
                    self.statReg += 1
                # Parse temperature
                elif self.statReg == 0x40:
                    value = value / 100
                    self.set(str(self.statReg), value)
                    self.statReg += 1
                # Other registers
                else:
                    self.set(str(self.statReg), value)
                    self.statReg += 1
            self.TempBytes.clear()

    # endregion

    # Send data over serial port
    def sendData(self, data):
        try:
            self.serialPort.write(data)
        except Exception as ex:
            print(ex)

    # Read registers
    def readReg(self, regAddr, regCount):
        # Store the starting register (needed when processing the response)
        self.statReg = regAddr
        # Build the read command frame and send it over the serial port
        self.sendData(self.get_readBytes(self.ADDR, regAddr, regCount))

    # Write register
    def writeReg(self, regAddr, sValue):
        # Unlock device registers
        self.unlock()
        # Delay 100ms
        time.sleep(0.1)
        # Build the write command frame and send it over the serial port
        self.sendData(self.get_writeBytes(self.ADDR, regAddr, sValue))
        # Delay 100ms
        time.sleep(0.1)
        # Save settings
        self.save()

    # Build read command frame
    def get_readBytes(self, devid, regAddr, regCount):
        # Initialize 8-byte buffer
        tempBytes = [None] * 8
        # Device Modbus address
        tempBytes[0] = devid
        # Read function code (0x03)
        tempBytes[1] = 0x03
        # Register address high 8 bits
        tempBytes[2] = regAddr >> 8
        # Register address low 8 bits
        tempBytes[3] = regAddr & 0xFF
        # Register count high 8 bits
        tempBytes[4] = regCount >> 8
        # Register count low 8 bits
        tempBytes[5] = regCount & 0xFF
        # Calculate CRC checksum
        tempCrc = self.get_crc(tempBytes, len(tempBytes) - 2)
        # CRC checksum high 8 bits
        tempBytes[6] = tempCrc >> 8
        # CRC checksum low 8 bits
        tempBytes[7] = tempCrc & 0xFF
        return tempBytes

    # Build write command frame
    def get_writeBytes(self, devid, regAddr, sValue):
        # Initialize 8-byte buffer
        tempBytes = [None] * 8
        # Device Modbus address
        tempBytes[0] = devid
        # Write function code (0x06)
        tempBytes[1] = 0x06
        # Register address high 8 bits
        tempBytes[2] = regAddr >> 8
        # Register address low 8 bits
        tempBytes[3] = regAddr & 0xFF
        # Register value high 8 bits
        tempBytes[4] = sValue >> 8
        # Register value low 8 bits
        tempBytes[5] = sValue & 0xFF
        # Calculate CRC checksum
        tempCrc = self.get_crc(tempBytes, len(tempBytes) - 2)
        # CRC checksum high 8 bits
        tempBytes[6] = tempCrc >> 8
        # CRC checksum low 8 bits
        tempBytes[7] = tempCrc & 0xFF
        return tempBytes

    # Start loop reading
    def startLoopRead(self):
        # Enable loop reading flag
        self.loop = True
        # Start polling thread
        t = threading.Thread(
            target=self.loopRead, args=()
        )  # Start a thread to poll data
        t.start()

    # Loop reading thread
    def loopRead(self):
        print("Loop reading started")
        while self.loop:
            self.readReg(0x3A, 13)
            time.sleep(0.2)
        print("Loop reading stopped")

    # Stop loop reading
    def stopLoopRead(self):
        self.loop = False

    # Unlock registers
    def unlock(self):
        cmd = self.get_writeBytes(self.ADDR, 0x69, 0xB588)
        self.sendData(cmd)

    # Save configuration
    def save(self):
        cmd = self.get_writeBytes(self.ADDR, 0x00, 0x0000)
        self.sendData(cmd)
