import Hiwonder
import time
import kinematics
from Hiwonder import LSC


def start_main():
  kinematics.set_link_length(6.9,9.5,9.5,16.9)
  LSC.moveServo(1,100,500)
  LSC.moveServo(2,500,500)
  kinematics.ki_move(0,17,20.5,0,1000)
  time.sleep(2)
  while True:
    left90()
    time.sleep(1)
    left45()
    time.sleep(1)
    right45()
    time.sleep(1)
    right90()
    time.sleep(1)

def left90():
  kinematics.ki_move(0,17,1.2,-71,800)    #机械臂向下探
  time.sleep(0.6)
  LSC.moveServo(1,500,500)                #机械爪收拢
  time.sleep(1)
  kinematics.ki_move(0,17,20.5,0,800)     #机械臂抬起复位
  time.sleep(0.6)
  kinematics.ki_move(-17,0,20.5,0,800)    #向左转
  time.sleep(0.6)
  kinematics.ki_move(-19.5,0,2.8,-60,800) #向左放下90度
  time.sleep(0.6)
  LSC.moveServo(1,100,500)                #机械爪放开
  time.sleep(1)
  kinematics.ki_move(-17,0,20.5,0,800)    #机械臂抬起
  time.sleep(0.6)
  kinematics.ki_move(0,17,20.5,0,800)     #机械臂转回中间位置复位
  time.sleep(0.6)

def right45():
  kinematics.ki_move(0,17,1.2,-71,800)
  time.sleep(0.6)
  LSC.moveServo(1,500,500)
  time.sleep(1)
  kinematics.ki_move(0,17,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(12,12,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(14,14,2.9,-59,800)
  time.sleep(0.6)
  LSC.moveServo(1,100,500)
  time.sleep(1)
  kinematics.ki_move(12,12,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(0,17,20.5,0,800)
  time.sleep(0.6)

def left45():
  time.sleep(2)
  kinematics.ki_move(0,17,1.2,-71,800)
  time.sleep(0.6)
  LSC.moveServo(1,500,500)
  time.sleep(1)
  kinematics.ki_move(0,17,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(-12,12,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(-14,14,2.9,-59,800)
  time.sleep(0.6)
  LSC.moveServo(1,100,500)
  time.sleep(1)
  kinematics.ki_move(-12,12,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(0,17,20.5,0,800)
  time.sleep(0.6)

def right90():
  kinematics.ki_move(0,17,1.2,-71,800)
  time.sleep(0.6)
  LSC.moveServo(1,500,500)
  time.sleep(1)
  kinematics.ki_move(0,17,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(17,0,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(19.5,0,2.8,-60,800)
  time.sleep(0.6)
  LSC.moveServo(1,100,500)
  time.sleep(1)
  kinematics.ki_move(17,0,20.5,0,800)
  time.sleep(0.6)
  kinematics.ki_move(0,17,20.5,0,800)
  time.sleep(0.6)

Hiwonder.startMain(start_main)
