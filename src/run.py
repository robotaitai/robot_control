#! /usr/bin/python3
import serial
import time
import rospy
from sensor_msgs.msg import Joy
from threading import Thread




class controller:
    def __init__(self) -> None:
        rospy.init_node('neo_control', anonymous=True)
        self.serial = serial.Serial('/dev/ttyTHS0', 9600, timeout=0.5)
        self.sub = rospy.Subscriber("/joy", Joy, self.callback)
        self.left_f = [0x8c, 60]
        self.right_f = [0x8A, 60]
        self.left_b = [0x8e, 60]
        self.right_b = [0x89, 60]
        self.left_i = [0x86, 0]
        self.right_i = [0x87, 0]
        self.state = "idle"

        self.left_command = 0
        self.right_command = 0
        self.tank_control = Thread(target=self.tank)
        self.tank_control.start()

    def run(self):
        rospy.spin()

    def tank(self):
        while True:
            if self.state == "idle":
                self.left_command = self.left_i
                self.right_command = self.right_i
            elif self.state == "forward":
                self.left_command = self.left_f
                self.right_command = self.right_f 
            elif self.state == "backward":
                self.left_command = self.left_b
                self.right_command = self.right_b 
            elif self.state == "right":
                self.left_command = self.left_f
                self.right_command = self.right_b 
            elif self.state == "left":
                self.left_command = self.left_b
                self.right_command = self.right_f 
            self.serial.write(self.left_command)
            time.sleep(0.01)
            self.serial.write(self.right_command)
            time.sleep(0.01)

    def callback(self, msg):
        # print(msg)
        if msg.axes[1] == 1.0:
            self.state = "forward"
        if msg.axes[1] == -1.0:
            self.state = "backward"
        if msg.axes[0] == -1.0: #RIGHT
            self.state = "right"
        if msg.axes[0] == 1.0: #LEFT
            self.state = "left"
        if msg.axes[0] == 0 and msg.axes[1] ==0:
            self.state = "idle"
        print(self.state)

control = controller()
control.run()
