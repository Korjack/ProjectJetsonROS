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
        self.read_padstate = None
        self.left_stick_level = 0
        self.right_stick_level = 0
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
        a = re.compile(".*?-event-joystick")
        for i in os.listdir(path):
            if a.match(i): return i
        return ""
    
    def connection_check(self):
        if self.gamepad == None:
            try: 
                self.gamepad = InputDevice("/dev/input/by-id/" + self.line_to_filename())
                print("Connected joy stick info:" + str(self.gamepad))
            except: self.gamepad = None

        try: self.read_padstate = self.gamepad.read_loop() 
        except:
            self.gamepad = None
            print("joystick pad lost, keep connection check.")
            time.sleep(1)

    def drive_accel(self):
        # left = UPDOWN, right = LEFTRIGHT

        # 강제 전진 방향 설정
        vector_x = 1; vector_y = 1
        motor_vector_l = 0; motor_vector_r = 0
        motor_level_l = 0; motor_level_r = 0
        
        if self.left_stick_level < 0: vector_x = 0
        elif self.left_stick_level > 0: vector_x = 1

        if self.right_stick_level < 0: vector_y = 0
        elif self.right_stick_level > 0: vector_y = 1

        if self.left_stick_level == 0 and self.right_stick_level == 0: return 0, 0
        
        if self.left_stick_level == 0 and self.right_stick_level != 0:
            if vector_x:
                motor_vector_l = 0
                motor_vector_r = 1
            else:
                motor_vector_l = 0
                motor_vector_r = 1
            motor_level_l = self.right_stick_level/2
            motor_level_r = self.right_stick_level/2
        else:
            if self.right_stick_level <= self.left_stick_level:
                if vector_x:
                    motor_level_l = self.left_stick_level
                    motor_level_r = self.left_stick_level-self.right_stick_level/1.2
                else:
                    motor_level_l = self.left_stick_level-self.right_stick_level/1.2
                    motor_level_r = self.left_stick_level
            else:
                if vector_x:
                    motor_level_l = self.left_stick_level
                    motor_level_r = self.left_stick_level-self.left_stick_level/1.2
                else:
                    motor_level_l = self.left_stick_level-self.left_stick_level/1.2
                    motor_level_r = self.left_stick_level

        
        return motor_level_l*(-motor_vector_l), motor_level_r*(-motor_vector_r)

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
                            self.left_stick_level = event.value + 32767
                        elif event.code == self.RIGHT_STICK_LEFTRIGHT:
                            self.right_stick_level = event.value + 32767
                except: self.gamepad = None

if __name__ == "__main__":
    stick_ctrl = joystick_control()
    time.sleep(20)
    stick_ctrl.thread_run = False
    stick_ctrl.receive_thread.join()