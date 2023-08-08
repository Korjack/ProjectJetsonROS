#!/usr/bin/env python3

#sudo apt install python3-evdev
from evdev import InputDevice, categorize, ecodes, KeyEvent
import time, threading
import os, re

class joystick_control:
    def __init__(self):    
        self.gamepad = None
        self.read_padstate = None
        self.thread_run = True

        self.left_stick_level = 0
        self.right_stick_level = 0
        self.AutoMode = False
        self.key_list_init()

        self.receive_thread = threading.Timer(0.5, self.stick_run)
        self.receive_thread.start()
    
    def key_list_init(self): ## KEY LIST Event Code ##
        ## STICK ##
        self.LEFT_STICK_UPDOWN = 1
        self.LEFT_STICK_LEFTRIGHT = 0
        self.RIGHT_STICK_UPDOWN = 4
        self.RIGHT_STICK_LEFTRIGHT = 3
        self.LEFT_STICKT_BUTTON = 317
        self.RIGHT_STICK_BUTTON = 318

        ## ARROW ##
        self.UPDOWN = 17
        self.LEFTRIGHT = 16

        ## BUTTON ##
        self.A = 304
        self.B = 305
        self.X = 307
        self.Y = 308
        self.LB = 310
        self.RB = 311
        self.BACK = 314
        self.START = 315

        ## TRIGER ##
        self.LT = 2
        self.RT = 5

    def line_to_filename(self):
        path = "/dev/input/by-id"
        r = re.compile(".*?-event-joystick")
        for i in os.listdir(path):
            if r.match(i): return i
        return ""
    
    def connection_check(self):
        if self.gamepad == None:
            try: 
                self.gamepad = InputDevice("/dev/input/by-id/" + self.line_to_filename())
                print("Connected joy stick info:" + str(self.gamepad))
            except: self.gamepad = None

        try: 
            self.read_padstate = self.gamepad.read_loop() 
        except KeyboardInterrupt:
            self.gamepad = None
            self.killing_thread()
        except:
            self.gamepad = None
            print("joystick pad lost, keep connection check.")
            time.sleep(1)

    def drive_accel(self):
        # left = UPDOWN, right = LEFTRIGHT
        # ML = 전후-(조향/2)
        # MR = 전후+(조향/2) (32767 to 100)
        motor_level_left = 0
        motor_level_right = 0
        
        if (self.right_stick_level/327.67) > 0:
            motor_level_left = (-self.left_stick_level/327.67)
            motor_level_right = (-self.left_stick_level/327.67)-((abs(self.right_stick_level)/327.67)/2)
        elif (self.right_stick_level/327.67) < 0:
            motor_level_left = (-self.left_stick_level/327.67)-((abs(self.right_stick_level)/327.67)/2)
            motor_level_right = (-self.left_stick_level/327.67)
        elif (self.left_stick_level/327.67) == 0: # LS 값이 0
            if (self.right_stick_level/327.67) > 0: # RS 값이 양수
                motor_level_left = (self.left_stick_level/327.67)
                motor_level_right = (-self.left_stick_level/327.67)
            elif (self.right_stick_level/327.67) < 0: # RS 값이 음수
                motor_level_left = (-self.left_stick_level/327.67)
                motor_level_right = (self.left_stick_level/327.67)
        else:
                motor_level_left = (-self.left_stick_level/327.67)
                motor_level_right = (-self.left_stick_level/327.67)

        return int(motor_level_left), int(motor_level_right)

    def killing_thread(self):
        self.read_padstate = None
        self.thread_run = False

    def stick_run(self):
        while(self.thread_run):
            self.connection_check()
            if self.gamepad != None:
                try:
                    for event in self.read_padstate:
                        if event.code == self.LEFT_STICK_UPDOWN:
                            self.left_stick_level = event.value
                        elif event.code == self.RIGHT_STICK_LEFTRIGHT:
                            self.right_stick_level = event.value
                        elif event.code == self.X:
                            self.AutoMode = False
                        elif event.code == self.Y:
                            self.AutoMode = True
                except: self.gamepad = None

if __name__ == "__main__":
    stick_ctrl = joystick_control()
    for k in range(10):
        time.sleep(1)
        stick_ctrl.left_stick_level += 3000; stick_ctrl.right_stick_level = 32767
        print("IN:" + str(stick_ctrl.left_stick_level) + "IN2:" + str(stick_ctrl.right_stick_level))
        print(str(stick_ctrl.drive_accel()))

    stick_ctrl.thread_run = False
    raise Exception("Stop.")
    stick_ctrl.receive_thread.join()