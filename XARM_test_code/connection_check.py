import xarm
import time

arm = xarm.Controller("USB")

print("Connected to xArm")
print("Voltage:", arm.getBatteryVoltage())

input("Press Enter to move servo 3 to position 500...")

arm.setPosition(1, 0)
time.sleep(3)
# 1 is claw opening
#2 is rotating claw
#3 is wrist
#4 is elbow
#5


print("First command sent")

input("Press Enter to move servo 3 to position 520...")

arm.setPosition(4, 520)
time.sleep(3)

print("Second command sent")