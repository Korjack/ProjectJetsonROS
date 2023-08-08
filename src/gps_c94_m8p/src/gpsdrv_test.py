#!/usr/bin/env python3

#<node pkg = "gpsdrv" name ="gps_c94_m8p" type ="gpsdrv.py" args="ttyACM0 gps_c94_m8p" output="screen"/>

#string getTime = GPS 최신 업데이트 시간값 가져옴. (이전의 getTime과 새로 수신된 getTime값이 같으면 GPS장치 연결 문제로 인식 시킬 것)
#int32 fixedstate = 0(Invalid), 1(2D/3D), 2(DGNSS), 4(Fixed RTK), 5(Float RTK), 6(Dead Reckoning)
#float64 position_error => fixedstate값이 4또는 5가 될때에 신뢰 할 것!

import sys, rospy, time, serial, threading, math
from gps_c94_m8p.msg import gps_data
from pyproj import Proj

class gilbot_pubsub:
    def __init__(self, comport="ttyACM0", nodeName="gps_c94_m8p"):
        self.nodeName = nodeName
        self.comm = comport
        self.serial_port = serial.Serial(
            port="/dev/" + self.comm,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0)
        
        self.rx_frames = []
        self.thread_run = True
        self.system_shutdown = False

        self.e_off_set = 0.0
        self.n_off_set = 0.0
        self.old_heading_rad = 0.0
        self.old_heading_degrees = 0.0
        self.old_utm_x = 0.0
        self.old_utm_y = 0.0

        #rospy.init_node(self.nodeName, anonymous=True)
        self.pub_gps_status =       rospy.Publisher("/gps_fix/", gps_data, queue_size=2)
        self.gps_status_msg = gps_data()
        self.proj_UTM = Proj(proj = 'utm', zone = 52, ellps = 'WGS84', preserve_units = False)

        self.rate = rospy.Rate(1)
        rospy.loginfo("Start / Port:%s / Baudrate:%d / Node: %s"%(self.comm, self.serial_port.baudrate, self.nodeName))
        
        if not rospy.is_shutdown():
            self.receive_thread = threading.Timer(0, self.serial_threading_rx)
            self.receive_thread.start()
            time.sleep(0.5) #port connetion check global deley.
        self.pub_sender_loop()

        self.sys_stop()
        self.print_log("Keeping main thread for other multiple thread join.....", end="")
        self.receive_thread.join()
        self.print_log("Complite.")
    
    def print_log(self, str_data, end="\r\n"):
        print(str_data, end=end)
        #rospy.loginfo(str_data)

    def cal_gps_heading(self, new_utm_x, new_utm_y):
        distance_limit = 0.4 #recalculrate heading, Unit: Meter
        heading_rad = self.old_heading_rad
        heading_degrees = self.old_heading_degrees
        delta_x = new_utm_x - self.old_utm_x
        delta_y = new_utm_y - self.old_utm_y
        
        if abs(delta_x) < distance_limit and abs(delta_y) < distance_limit: return heading_rad, heading_degrees
        if abs(delta_x) < self.gps_status_msg.position_error and abs(delta_y) < self.gps_status_msg.position_error: return heading_rad, heading_degrees
        if delta_x == 0.0 and delta_y == 0.0: return heading_rad, heading_degrees
        
        heading_rad = math.atan2(delta_y, delta_x)
        
        degrees = heading_rad * 180 / math.pi
        if degrees > 90: degrees = 450 - degrees
        else: degrees = 90 - degrees
        heading_degrees = degrees

        self.old_heading_rad = heading_rad
        self.old_heading_degrees = heading_degrees

        self.old_utm_x = new_utm_x
        self.old_utm_y = new_utm_y
        
        return heading_rad, heading_degrees

    def pub_sender_loop(self):
        while self.thread_run and not rospy.is_shutdown():
            self.rate.sleep()

    def gps_publish(self):
        try:
            xy_zone = self.proj_UTM(self.gps_status_msg.longitude, self.gps_status_msg.latitude)
            self.gps_status_msg.utm_x = xy_zone[0]-self.e_off_set
            self.gps_status_msg.utm_y = xy_zone[1]-self.n_off_set
            rad, degree = self.cal_gps_heading(self.gps_status_msg.utm_x, self.gps_status_msg.utm_y)
            self.gps_status_msg.heading_rad = rad
            self.gps_status_msg.heading_degrees = degree
        except KeyboardInterrupt:
            self.print_log("Keyboard exception !")
            self.sys_stop()
        except Exception as e:
            self.print_log(str(e))
        finally:
            self.pub_gps_status.publish(self.gps_status_msg)
        
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
                    if c == 10:
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
        self.print_log("Close gps driver rx thread.")
        self.sys_stop()
    
    def serial_command_receive(self, rx_frame):
        split_data = rx_frame.split(',')
        split_data[2] = split_data[2].replace("\r", "")
        split_data[2] = split_data[2].replace("\n", "")
        vector_yaw = float(str(split_data[2]))
        print(round(vector_yaw, 1))
        return
        try:
            if split_data[0] == "$GNGGA" and len(split_data) >= 15:
                #[1:UTC Time], [2:Latitude] [3:NorIndi] [4:Longtitude] [5:EastIndi] [6:FixedStatus] [7:SVsUsed] [8:HDOP] [9:Altitude] [10:AltiUnit] [11:GeoidSep] [12:GeoidUnit] [13:AgeOfDGNSSCorr] [14:DGNSSRefStation]
                self.gps_status_msg.getTime = self.parse_now_time(split_data[1])
                self.gps_status_msg.latitude = self.parse_lati_longti(split_data[2],split_data[3])
                self.gps_status_msg.longitude = self.parse_lati_longti(split_data[4],split_data[5])
                self.gps_status_msg.fixedstate = int(split_data[6])
                self.gps_status_msg.altitude = float(split_data[9])
                # print("Position: ", self.gps_status_msg.latitude, self.gps_status_msg.longitude)
                # print("Position: ", split_data[2], split_data[4])
                
            elif split_data[0] == "$GNGST" and len(split_data) >= 9:
                #[1:UTC Time], [2:RMS] [3:StdDevMaj] [4:StdDevMin] [5:Orientation] [6:StdDevLat] [7:StdDevLon] [8:StdDevAlt]
                self.gps_status_msg.getTime = self.parse_now_time(split_data[1])
                latError = float(split_data[6])
                lonError = float(split_data[7])
                lastSplit = split_data[8].split("*")
                altError = float(lastSplit[0])
                self.gps_status_msg.position_error = latError
                if self.gps_status_msg.position_error < lonError: self.gps_status_msg.position_error = lonError
                if self.gps_status_msg.position_error < altError: self.gps_status_msg.position_error = altError
                self.gps_publish()

        except: pass
        
    def parse_lati_longti(self, value, dir):
        deg = 0
        fl = 0.0
        if dir == "N":
            deg = int(value[:2])
            fl = float(value[2:])
        elif dir == "E":
            deg = int(value[:3])
            fl = float(value[3:])
        #print('print', value, 'deg', deg, 'fl', fl)
        longti = deg + fl / 60
        #print('longti', longti)
        return float(longti)
    
    def parse_now_time(self, data):
        hour, minute, sec, fl = data[0:2], data[2:4], data[4:6], data[6:]
        #print(hour, ":", minute, ":", sec + ".", fl)
        return str(int(hour) + 9) + minute + sec + fl

if __name__=='__main__':
    gps_active = None
    start_port = "ttyUSB1"
    node_name = "gps_c94_m8p"

    try: start_port = str(sys.argv[1])
    except: start_port = "ttyUSB1" #default serial port

    try: node_name = str(sys.argv[2])
    except: node_name = "gps_c94_m8p" #default node name

    try:
        rospy.init_node(node_name, anonymous = True)
        gps_active = gilbot_pubsub(start_port, node_name)
    except rospy.ROSInterruptException:
        gps_active.sys_stop()
        pass
