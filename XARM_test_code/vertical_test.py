import Hiwonder
import time
import kinematics
from Hiwonder import LSC


def start_main():
  kinematics.set_link_length(6.9,9.5,9.5,16.9)  #设置机械臂连杆长度 # Set robotic arm lengths
  LSC.moveServo(1,100,1000)   #控制1、2号舵机转动 # Control servos 1 and 2
  LSC.moveServo(2,500,1000)
  kinematics.ki_move(0,17,20.5,0,1500)  #控制机械臂运动至初始姿态 # Set robotic arms to initial position
  time.sleep(1.5)
  while True:
    kinematics.ki_move(0,17,16.4,0,1500)   #使机械臂竖直向下运动 # Move robotic arm down vertically
    time.sleep(1.5)
    kinematics.ki_move(0,17,25.8,0,1500)   #使机械臂竖直向上运动 # Move robotic arm up vertically
    time.sleep(1.5)

Hiwonder.startMain(start_main)
