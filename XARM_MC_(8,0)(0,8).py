import xarm
import time

arm = xarm.Controller('USB')
print("Battery Voltage:", arm.getBatteryVoltage())

CLAW_OPEN = 100
CLAW_CLOSE = 700
Z_HOVER = (300,680,500)
Z_GROUND = (680,520,350)
COORD_8_0 = 490
COORD_0_8 = 850

def move_horizontal(target_xy):
    arm.setPosition(3, Z_HOVER[0])
    arm.setPosition(4, Z_HOVER[1])
    arm.setPosition(5, Z_HOVER[2])
    arm.setPosition(6, target_xy)
    time.sleep(2)

def move_vertical_z(height_pose):
    arm.setPosition(3, height_pose[0])
    arm.setPosition(4, height_pose[1])
    arm.setPosition(5, height_pose[2])
    time.sleep(2)

#(8,0) to (0,8)
arm.setPosition(1, CLAW_OPEN)
move_horizontal(COORD_8_0)
move_vertical_z(Z_GROUND)
arm.setPosition(1, CLAW_CLOSE)
time.sleep(2)

move_vertical_z(Z_HOVER)
move_horizontal(COORD_0_8)
move_vertical_z(Z_GROUND)
arm.setPosition(1, CLAW_OPEN)
time.sleep(2)

move_vertical_z(Z_HOVER)
