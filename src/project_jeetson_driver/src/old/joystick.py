#!/usr/bin/python3
import struct, threading
import math

class joystick(threading.Thread):
    def __init__(self, path="/dev/input/js0", deadzone=0, stick_scale=1, triger_scale=1):
        threading.Thread.__init__(self)
        self.path = path
        self.deadzone = deadzone
        self.stick_scale = stick_scale
        self.triger_scale = triger_scale
        self.push_time = 0
        self.isDouble = False
        self.joystick = {"Axes": [0, 0, -32767, 0, 0, -32767, 0, 0],
                         "Buttons": [False, False, False, False, False, False, False, False, False, False, False]}
        self.isEnd = False

    def read_joystick(self):
        with open(self.path, "rb") as f:
            while not self.isEnd:
                a = f.read(8)
                t, value, code, index = struct.unpack("<Ihbb", a) # 4 bytes, 2 bytes, 1 byte, 1 byte
                t = self.push_time
                if code == 2:
                    if value > self.deadzone:
                        value -= self.deadzone
                    elif value < -self.deadzone:
                        value += self.deadzone
                    else:
                        value = 0

                    self.joystick["Axes"][index] = value

                if code == 1:
                    self.joystick["Buttons"][index] = True if value else False

    def read_stickValue(self):
        while not self.isEnd:
            if self.joystick["Buttons"][5] and not self.isDouble:
                self.isDouble = True
            elif self.joystick["Buttons"][5] and self.isDouble:
                self.isDouble = False

    def get_Left_Wheel(self):
        value = 0
        if self.joystick["Axes"][0] > self.deadzone and self.joystick["Axes"][0] < -self.deadzone:
            value = math.trunc((self.joystick["Axes"][2]) * self.stick_scale)
        else:
            value += math.trunc((self.joystick["Axes"][5] + 32767) * self.triger_scale)
            value -= math.trunc((self.joystick["Axes"][2] + 32767) * self.triger_scale)
        if self.isDouble:
            value *= 2
        return value
    
    def get_Right_Wheel(self):
        value = 0
        if self.joystick["Axes"][0] > self.deadzone and self.joystick["Axes"][0] < -self.deadzone:
            value = math.trunc((self.joystick["Axes"][5]) * self.stick_scale)
        else:
            value += math.trunc((self.joystick["Axes"][5] + 32767) * self.triger_scale)
            value -= math.trunc((self.joystick["Axes"][2] + 32767) * self.triger_scale)
        if self.isDouble:
            value *= 2
        return value

    def get_joystick(self):
        return getattr(self, "joystick")
    
    def run(self):
        th = threading.Timer(1, self.read_stickValue)
        th.start()
        self.read_joystick()

if __name__ == "__main__":
    js = joystick()
    while True:
        try:
            print(js.get_joystick())
        except KeyboardInterrupt:
            js.isEnd = True