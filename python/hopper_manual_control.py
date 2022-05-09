import numpy as np
import time
import struct

import hid
import serial
from serial.tools import list_ports

LINEAR_VELOCITY_SCALE = 40.0
ANGULAR_VELOCITY_SCALE = 3.0

BAUDRATE = 115200
COMMAND_RATE = 20

class SerialSender:
    def __init__(self, port, baudrate, start='H'):
        self.port = port
        self.baudrate = baudrate
        self.start = bytes(start, 'ascii')

        self.ser = serial.Serial(port, baudrate)
        
    def send(self, linear_velocity, angular_velocity):
        data = struct.pack("<cff", self.start, linear_velocity, angular_velocity)
        self.ser.write(data)
        
    def open(self):
        self.ser.open()
    
    def close(self):
        self.ser.close()

    def read_byte(self):
        return self.ser.read()

    def read_float(self):
        return struct.unpack("<f", self.ser.read(4))[0]

    def read_int(self):
        return struct.unpack("<i", self.ser.read(4))[0]

class JoystickInterface:
    def __init__(self):
        self.gamepad = hid.device()
        for device in hid.enumerate():
            if (device['product_string'] == 'FrSky Simulator') or (device['product_string'] == 'BetaFPV Taranis Joystick') or (device["vendor_id"] == 1155):
                self.gamepad.open(device['vendor_id'], device['product_id'])
                break
        self.gamepad.set_nonblocking(True)

    def get_command(self):
        report = None
        while not report:
            report = self.gamepad.read(64)
        event = [r - 256 if r > 127 else r for r in report]
        command_linear_vel = LINEAR_VELOCITY_SCALE * event[4] / 127
        command_angular_vel = ANGULAR_VELOCITY_SCALE * event[6] / 127
        return command_linear_vel, command_angular_vel


if __name__ == "__main__":
    
    try: 
        port = next(list_ports.grep(".*usb*"))
    except:
        try:
            port = next(list_ports.grep(".*tty.*"))
        except:
            raise RuntimeError("No port found")

    ser = SerialSender(port.device, BAUDRATE)
    joy = JoystickInterface()

    while True:
        time.sleep(1 / COMMAND_RATE)
        lin, ang = joy.get_command()
        ser.send(lin, ang)
        # print(ser.read_byte())
        # print(ser.read_float())
        # print(ser.read_float())