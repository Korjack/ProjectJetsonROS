#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Int32
#This file is user control lib, copy to coding workspace.
class JetsonBotControlNode:
    def __init__(self, nodeName="motor_driver_control", demo_mode=False):
        self.nodeName = nodeName

        #driver control variables
        self.Cset_motor_l = 0
        self.Cset_motor_r = 0
        #driver reading variables
        self.Cget_connection = 0
        self.Cget_rotor_hall = 0
        self.Cget_rotor_hall_l = 0
        self.Cget_rotor_hall_r = 0

        #ROS init start
        self.state_view = True
        self.classed_mode = False
        try: rospy.init_node(self.nodeName, anonymous=True)
        except:
            print("Error node init car client node import mode detected.")
            self.classed_mode = True

        self.pub_motor_left =       rospy.Publisher("/command/throttle_left", Int32, queue_size=1)
        self.pub_motor_right =      rospy.Publisher("/command/throttle_right", Int32, queue_size=1)
        
        self.sub_car_connected =        rospy.Subscriber("/value/car_connected", Int32, self.callback_connection)
        self.sub_rotor_hall =           rospy.Subscriber("/value/rotor_hall", Int32, self.callback_rotor_hall)
        self.sub_rotor_hall_l =         rospy.Subscriber("/value/rotor_hall_l", Int32, self.callback_rotor_hall_l)
        self.sub_rotor_hall_r =         rospy.Subscriber("/value/rotor_hall_r", Int32, self.callback_rotor_hall_r)

        rospy.loginfo("Start motor driver / Node: %s"%(self.nodeName))
        self.rate = rospy.Rate(10)

        if not demo_mode:
            if not self.classed_mode:
                while not rospy.is_shutdown(): #for this coding
                    #get user code here.
                    self.rate.sleep()
        else:
            ######################### demo code ###########################
            test_cycle = 50
            while not rospy.is_shutdown():
                for a in range(test_cycle):
                    self.Cset_motor_l = 30
                    self.Cset_motor_r = 30
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()

                for a in range(test_cycle):
                    self.Cset_motor_l = 80
                    self.Cset_motor_r = 80
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()

                for a in range(test_cycle):
                    self.Cset_motor_l = -50
                    self.Cset_motor_r = -50
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()

                for a in range(test_cycle):
                    self.Cset_motor_l = -50
                    self.Cset_motor_r = 50
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()
                
                for a in range(test_cycle):
                    self.Cset_motor_l = 50
                    self.Cset_motor_r = -50
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()
                
                for a in range(test_cycle):
                    self.Cset_motor_l = 0
                    self.Cset_motor_r = 0
                    self.Cset_active() #driver active lost check
                    self.rate.sleep()
                self.get_state_view()
                ######################### demo code ###########################
        
        if not self.classed_mode: print("Terminate. car driving control.")  

    def callback_connection(self, msg): self.Cget_connection = int(msg.data)
    def callback_rotor_hall(self, msg): self.Cget_rotor_hall = int(msg.data)
    def callback_rotor_hall_l(self, msg): self.Cget_rotor_hall_l = int(msg.data)
    def callback_rotor_hall_r(self, msg): self.Cget_rotor_hall_r = int(msg.data)

    def get_state_view(self):
        if self.state_view:
            print("---------------client side--------------")
            print("DRVCONN(%d)"%(self.Cget_connection))
            print("ROTOR_HA(%04d)(L:%04d R:%04d)"%(self.Cget_rotor_hall, self.Cget_rotor_hall_l, self.Cget_rotor_hall_r))

    def Cset_active(self):
        self.pub_motor_left.publish(int(self.Cset_motor_l))
        self.pub_motor_right.publish(int(self.Cset_motor_r))

if __name__=='__main__':
    driverControl = None
    try:
        driverControl = JetsonBotControlNode("motorDrv", demo_mode=True)
    except rospy.ROSInterruptException:
        pass
    driverControl.demo_thread_run = False