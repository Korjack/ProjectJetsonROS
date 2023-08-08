#!/usr/bin/env python3
import rospy, cv2, sys
from ref_driver_use_node import JetsonBotControlNode
from ref_joystick import joystick_control

from std_msgs.msg import Bool
from jetson_msg.msg import driving_msg
from em_brake.msg import em_data

class start_main:
    def __init__(self):
        rospy.init_node("control_main", anonymous=True)

        rospy.Subscriber("/ctrl_cmd", driving_msg, self.ctrl_cmd_callback) # 차량 제어 데이터
        rospy.Subscriber("/em_brake", em_data, self.emcallback) # 긴급정지 Node 확인
        rospy.Subscriber("/is_dest", Bool, self.destcallback) # 목적지 도달 확인

        self.motorDrv       = JetsonBotControlNode("motordrv") # Serial 통신으로 자동차 제어
        self.rate = rospy.Rate(20)
        self.joystick = joystick_control() # 조이스틱 데이터 수집

        self.left_accel = 0 # 왼쪽 바퀴 엑셀
        self.right_accel = 0 # 오른쪽 바퀴 엑셀
        self.em_flag = False
        self.is_arrived = False

        loopa = 0
        while not rospy.is_shutdown():
            if self.joystick.gamepad: # 조이스틱이 잡혔을 떄
                if self.joystick.AutoMode: # 자율주행상태일 때
                    if self.em_flag or self.is_arrived: # 차량이 긴급적으로 멈춰야하거나 목적지에 도착하여 정지해야할 때
                        self.motorDrv.Cset_motor_l = 0.0
                        self.motorDrv.Cset_motor_r = 0.0
                    else:
                        self.motorDrv.Cset_motor_l = self.left_accel
                        self.motorDrv.Cset_motor_r = self.right_accel
                else: # 자율주행 상태가 아닐 때
                    loopa += 1
                    drive_level_l, drive_level_r = self.joystick.drive_accel()
                    
                    if loopa >= 20:
                        loopa = 0
                        print(self.joystick.left_stick_level, self.joystick.right_stick_level, drive_level_l, drive_level_r)
                    
                    # 조이스틱의 입력값을 받아서 처리
                    self.motorDrv.Cset_motor_l = drive_level_l
                    self.motorDrv.Cset_motor_r = drive_level_r
            else: # 연결이 끊기면 바로 제자리 정지
                self.motorDrv.Cset_motor_l = 0.0
                self.motorDrv.Cset_motor_r = 0.0

            # print(self.joystick.get_Left_Wheel(), self.joystick.get_Right_Wheel())
            # self.motorDrv.Cset_motor_l = self.joystick.get_Left_Wheel()
            # self.motorDrv.Cset_motor_r = self.joystick.get_Right_Wheel()

            self.motorDrv.Cset_active()
            
            self.rate.sleep()
        self.joystick.killing_thread()
    
    def ctrl_cmd_callback(self, msg):
        self.left_accel = msg.left
        self.right_accel = msg.right

    def emcallback(self, msg):
        self.em_flag = msg.stop

    def destcallback(self, msg):
        self.is_arrived = msg.data

if __name__ == "__main__":
    control_main = None
    try: control_main = start_main()
    except rospy.ROSInterruptException:
        control_main.motorDrv.demo_thread_run = False
        control_main.joystick.killing_thread()