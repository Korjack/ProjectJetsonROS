#!/usr/bin/env python3
from os import replace
import sys, rospy, time, serial, threading
from std_msgs.msg import Int32

class jetsonBot_drvnode:
    def __init__(self, comport="ttyUSB0", nodeName="motor_driver"):
        self.nodeName = nodeName
        self.comm = comport
        self.serial_port = serial.Serial(
            port="/dev/" + self.comm,
            baudrate=38400,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0)
        
        self.rx_frames = []
        self.thread_run = True
        self.system_shutdown = False
        self.subscriber_timeout = 400
        
        self.sub_motor_left =       rospy.Subscriber("/command/throttle_left", Int32, self.callback_throttle_left)
        self.sub_motor_right =      rospy.Subscriber("/command/throttle_right", Int32, self.callback_throttle_right)
        
        self.pub_car_connected =        rospy.Publisher("/value/car_connected", Int32, queue_size=1)
        self.pub_rotor_hall =           rospy.Publisher("/value/rotor_hall", Int32, queue_size=1)
        self.pub_rotor_hall_l =         rospy.Publisher("/value/rotor_hall_l", Int32, queue_size=1)
        self.pub_rotor_hall_r =         rospy.Publisher("/value/rotor_hall_r", Int32, queue_size=1)
        
        self.rate = rospy.Rate(10)
        rospy.loginfo("Start / Port:%s / Baudrate:%d / Node: %s"%(self.comm, self.serial_port.baudrate, self.nodeName))
        
        #driver control variables
        self.Cset_motor_l = 0
        self.Cset_motor_r = 0

        #driver reading variables
        self.Cget_connection = 0
        self.Cget_rotor_hall = 0
        self.Cget_rotor_hall_l = 0
        self.Cget_rotor_hall_r = 0
        
         #timeout timer rospyRate 40Hz(25ms), 40cycle is 1 second.
        self.subscriber_timeout_cycle = 40
        
        if not rospy.is_shutdown():
            self.receive_thread = threading.Timer(0, self.serial_threading_rx)
            self.receive_thread.start()
            self.transmit_thread = threading.Timer(0.1, self.serial_threading_tx)
            self.transmit_thread.start()
            time.sleep(0.5) #port connetion check global deley.

        self.pub_sender_loop()
        self.sys_stop()
        self.print_log("Keeping main thread for other multiple thread join.....", end="")
        self.receive_thread.join()
        self.transmit_thread.join()
        self.print_log("Terminate complite.")
    
    def pub_sender_loop(self):
        while self.thread_run and not rospy.is_shutdown():
            try:
                self.pub_car_connected.publish(int(self.Cget_connection))
                self.pub_rotor_hall.publish(int(self.Cget_rotor_hall))
                self.pub_rotor_hall_l.publish(int(self.Cget_rotor_hall_l))
                self.pub_rotor_hall_r.publish(int(self.Cget_rotor_hall_r))

                if self.subscriber_timeout > 0: self.subscriber_timeout -= 1 #connection check variable timer cycle
                self.rate.sleep()
            except KeyboardInterrupt:
                self.print_log("Keyboard exception !")
                self.sys_stop()
            except:
                self.sys_stop()
    
    def callback_throttle_left(self, msg):
        self.Cset_motor_l = int(msg.data)
        self.subscriber_timeout = self.subscriber_timeout_cycle

    def callback_throttle_right(self, msg):
        self.Cset_motor_r = int(msg.data)
        self.subscriber_timeout = self.subscriber_timeout_cycle
    
    def driver_ctrl_reset(self):
        self.Cset_motor_l = 0
        self.Cset_motor_r = 0
        self.Cget_connection = 0
        
    def print_log(self, str_data, end="\r\n"):
        print(str_data, end=end)
        #rospy.loginfo(str_data)
        
    def sys_stop(self):
        self.thread_run = False
        if not self.system_shutdown:
            self.system_shutdown = True
            self.serial_port.cancel_read()
            self.serial_port.close()
            rospy.signal_shutdown("Node Close.")

    def serial_threading_rx(self):
        while self.thread_run and not rospy.is_shutdown():
            try:
                for c in self.serial_port.read():
                    self.rx_frames.append(chr(c))
                    if c == 13:
                        self.serial_command_receive(''.join(self.rx_frames))
                        del self.rx_frames[:]
                    elif len(self.rx_frames) > 255:
                        del self.rx_frames[:]
            except KeyboardInterrupt:
                self.print_log("Keyboard exception !")
                self.sys_stop()
            except Exception as e:
                self.print_log("Error occurred. Exiting Program: " + str(e))
                self.thread_run = False
        self.print_log("Close car driver rx thread.")
        self.sys_stop()
    
    def serial_command_receive(self, rx_frame):
        # 52+50+51+44+52+50+59+44 = 402+1
        # RX frame EX: 423,421,403[13]
        # 423 = Left hole count
        # 421 = Right hole count
        # 393 = checksum value for "423, 421,"
        # [13] = \r word (comm end signal)
        # checksum = [52+50+51+44+52+50+49+44 = 392+1]
        split_data = None
        try: split_data = rx_frame.split(',')
        except: return
        
        if len(split_data) != 3: return
        
        rx_checksum = int(split_data[len(split_data)-1])
        rx_checksum_cal = 1 #default 0 data checksum is 1
        
        #make received checksum data. (this checksum 4byte end lastword 1byte)
        for arp in range(len(split_data)-1):
            for word in split_data[arp]: rx_checksum_cal += ord(word)
            rx_checksum_cal += ord(',')

        #check to checksum
        if rx_checksum_cal == rx_checksum:
            if self.Cget_connection < 5: self.Cget_connection += 1
            self.Cget_rotor_hall_l = int(split_data[0])
            self.Cget_rotor_hall_r = int(split_data[1])
            self.Cget_rotor_hall = self.Cget_rotor_hall_l + self.Cget_rotor_hall_r
        else: self.print_log("checksum error. C:%d, R:%d"%(rx_checksum_cal, rx_checksum))

    def serial_threading_tx(self):
        #frame address0123456789.......
        #TX frame EX: N000N0000445[13]  (78+48+48+48+78+48+48+48)+1=445
        # 0: Drive Mode left ('F':Forward, 'R':Reverse, 'B':Break, else:Normal)
        # 1,2,3: Throttle level left (000~127)
        # 4: Drive Mode right('F':Forward, 'R':Reverse, 'B':Break, else:Normal)
        # 5,6,7: Throttle level right (000~127)
        # 8,9,10,11: ('N'78+'0'48+'0'48+'0'48+'N'78+'0'48+'0'48+'0'48)+1=0445
        # 12: last word [line feed, 13]

        tx_frame = ""
        fail_count = 0
        
        while self.thread_run and not rospy.is_shutdown():
            try:
                time.sleep(0.05)
                if fail_count >= 5:
                    self.print_log("send fail. limited.")
                    self.thread_run = False
                
                if self.subscriber_timeout == 0: self.driver_ctrl_reset()

                if self.Cset_motor_l < 0: tx_frame = "R"
                elif self.Cset_motor_l > 0: tx_frame = "F"
                else: tx_frame = "S"
                tx_frame += "%03d"%(abs(self.Cset_motor_l))

                if self.Cset_motor_r < 0: tx_frame += "R"
                elif self.Cset_motor_r > 0: tx_frame += "F"
                else: tx_frame += "S"
                tx_frame += "%03d"%(abs(self.Cset_motor_r))
                
                checksum = 1
                for c in tx_frame: 
                    checksum += ord(c)
                tx_frame += ("%04d\r" % checksum)
                
                self.serial_port.write(tx_frame.encode())
                fail_count = 0
            except KeyboardInterrupt:
                self.print_log("Keyboard exception !")
                self.sys_stop()
            except Exception as e:
                self.print_log("send fail. " + str(e))
                if fail_count < 5: fail_count += 1

        self.print_log("Close car driver tx thread.")
        self.sys_stop()

if __name__=='__main__':
    driver_active = None
    start_port = "ttyUSB0"
    node_name = "motor_driver"

    try: start_port = str(sys.argv[1])
    except: start_port = "ttyUSB0" #default serial port

    try: node_name = str(sys.argv[2])
    except: node_name = "motor_driver" #default node name

    try:
        rospy.init_node(node_name, anonymous = True)
        driver_active = jetsonBot_drvnode(start_port, node_name)
    except rospy.ROSInterruptException:
        driver_active.sys_stop()
        pass
