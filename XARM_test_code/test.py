import xarm
import time

arm1 = xarm.Controller("USB")
# arm2 = xarm.Controller("USB")
#
arm1.setPosition(2, 300)
time.sleep(1.5)
arm2.setPosition(3, 700)
import serial.tools.list_ports

for port in serial.tools.list_ports.comports():
    print(port.device, port.description, port.serial_number)
#
# arm connected to singular USB port, can connect multiple arms but needs to specify
arm = xarm.Controller('USB')

# print (arm.getBatteryVoltage())
arm.setPosition(2,700)
barm.setPosition(2,700)
# carm.setPosition(2,700)
# define servo1 assumes default position of 500
servo1 = xarm.Servo(1)
print('servo ID:', servo1.servo_id)
print('servo position:', servo1.position)
print('servo angle:', servo1.angle)

servo2 = xarm.Servo(2)
# Set servo ID 1 to position 100.
arm.setPosition(1, 300)

time.sleep(2)

# Set servo ID 1 to position 700
arm.setPosition(1, 700)


arm.setPosition(2,100, wait=True)
arm.setPosition(3,100, wait=True)
arm.setPosition(4,100, wait=True)
arm.setPosition(5,100, wait=True)
arm.setPosition(6,100, wait=True)

arm.setPosition(2,500, wait=True)
arm.setPosition(3,500, wait=True)
arm.setPosition(4,500, wait=True)
arm.setPosition(5,500, wait=True)
arm.setPosition(6,500, wait=True)

# sets servo3 to 45 degrees and waits the default 1 second
# before returning
arm.setPosition(3, 45.0, wait=True)
time.sleep(2)
arm.setPosition(3,90.0)
arm.setPosition(3,0.0)

def setPose(servo1, servo2, servo3, servo4, servo5, servo6):
    arm.setPosition(1, claw, wait=True)
    arm.setPosition(2,wrist, wait=True)
    arm.setPosition(3,elbow, wait=True)
    arm.setPosition(4,arm, wait=True)
    arm.setPosition(5,100, wait=True)
    arm.setPosition(6,100, wait=True)

# Move the end effector directly, instead of setting each servo independently does this work?
arm.set_position(
    x=300,
    y=100,
    z=200,
    roll=180,
    pitch=0,
    yaw=0,
    speed=100
)





