import numpy as np
import time

import hid
import serial

LINEAR_VELOCITY_SCALE = 10.0
ANGULAR_VELOCITY_SCALE = 10.0

class JoystickInterface:
    def __init__(self):
        self.gamepad = hid.device()
        for device in hid.enumerate():
            if (device['product_string'] == 'FrSky Simulator') or (device['product_string'] == 'BetaFPV Taranis Joystick'):
                self.gamepad.open(device['vendor_id'], device['product_id'])
                break
        self.gamepad.set_nonblocking(True)
        while True:
            self.get_command()

    def get_command(self):
        report = self.gamepad.read(64)
        if report:
            event = [r - 256 if r > 127 else r for r in report]
            command_linear_vel = LINEAR_VELOCITY_SCALE * event[4] / 127
            command_angular_vel = ANGULAR_VELOCITY_SCALE * event[6] / 127
            print(command_linear_vel, command_angular_vel)
            return

JoystickInterface()