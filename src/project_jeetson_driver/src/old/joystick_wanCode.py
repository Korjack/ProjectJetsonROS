#!/usr/bin/env python3

#sudo apt install python3-evdev
from evdev import InputDevice, categorize, ecodes, KeyEvent
import time

#gamepad = InputDevice('/dev/input/js0')
gamepad = None
read_padstate = None
while(True):
    if gamepad == None:
        gamepad = InputDevice('/dev/input/event4')
        print("?" + str(gamepad))
    
    try: read_padstate = gamepad.read_loop() 
    except:
        gamepad = None
        print("joystick pad lost, keep connection check.")
        time.sleep(1)
    
    if gamepad != None:
        for event in read_padstate: print(event)
    continue
    for event in gamepad.read_loop():
        print(event)
        if event.type == ecodes.EV_KEY:
            keyevent = categorize(event)
            if keyevent.keystate == KeyEvent.key_down:
                print(keyevent)
            elif keyevent.keycode == 'BTN_TL':
                print ("Guns")
        elif event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            #print(absevent.event.code)
            # print(absevent)
            print(event)
            # if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0X':
            #     if absevent.event.value == -1:
            #             print('left')
            #     elif absevent.event.value == 1:
            #             print('right')
            # if ecodes.bytype[absevent.event.type][absevent.event.code] == 'ABS_HAT0Y':
            #     if absevent.event.value == -1:
            #             print('forward')
            #     elif absevent.event.value == 1:
            #             print('back')
    print("joystick pad lost, keep connection check.")
    time.sleep(1)