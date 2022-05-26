import numpy as np
import time
import struct

import hid
import serial
from serial.tools import list_ports

LINEAR_VELOCITY_SCALE = 20.0
ANGULAR_VELOCITY_SCALE = 3.0
MAX_LINEAR_ACCEL = 60.0
MAX_ANGULAR_ACCEL = 3.0

LEG_HEIGHT_SCALE = 0.25
LEG_HEIGHT_OFFSET = 0.5
LEG_TILT_SCALE = 0.2

BAUDRATE = 115200
COMMAND_RATE = 20

JOYSTICK_DEADBAND = 0.05

class SerialSender:
    def __init__(self, port, baudrate, start='H'):
        self.port = port
        self.baudrate = baudrate
        self.start = bytes(start, 'ascii')

        self.ser = serial.Serial(port, baudrate)
        
    def send(self, linear_velocity, angular_velocity, leg_height, leg_tilt):
        data = struct.pack("<cffff", self.start, linear_velocity, angular_velocity, leg_height, leg_tilt)
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
            # print(device)
            if (device['product_string'] == 'FrSky Simulator') or (device['product_string'] == 'BetaFPV Taranis Joystick') or (device["vendor_id"] == 1155):
                self.gamepad.open(device['vendor_id'], device['product_id'])
                break
        self.gamepad.set_nonblocking(True)

    def get_command(self):
        report = None
        while not report:
            report = self.gamepad.read(64)
        event = [(r - 256) / 127.0 if r > 127 else r / 127.0 for r in report]
        # print(event)
        command_linear_vel = LINEAR_VELOCITY_SCALE * event[4] if abs(event[4]) > JOYSTICK_DEADBAND else 0
        command_angular_vel = ANGULAR_VELOCITY_SCALE * event[3] if abs(event[3]) > JOYSTICK_DEADBAND else 0
        command_leg_height = LEG_HEIGHT_SCALE * event[5] + LEG_HEIGHT_OFFSET
        command_leg_tilt = LEG_TILT_SCALE * event[6] if abs(event[6]) > JOYSTICK_DEADBAND else 0
        return command_linear_vel, command_angular_vel, command_leg_height, command_leg_tilt


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

    lin = 0
    ang = 0

    try:
        while True:
            time.sleep(1 / COMMAND_RATE)
            joy_lin, joy_ang, joy_height, joy_tilt = joy.get_command()
            if joy_lin - lin > 0:
                lin = min(joy_lin, lin + MAX_LINEAR_ACCEL / COMMAND_RATE)
            else:
                lin = max(joy_lin, lin - MAX_LINEAR_ACCEL / COMMAND_RATE)

            ang = joy_ang
            # print(lin, ang)
            ser.send(lin, ang, joy_height, joy_tilt)
            # print(ser.read_byte())
            # print(ser.read_float())
            # print(ser.read_float())
    finally:
        ser.send(0, 0, 0.6, 0)
