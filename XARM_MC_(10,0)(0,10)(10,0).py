import xarm
import time

arm = xarm.Controller('USB')
print (arm.getBatteryVoltage())

# 1.Set Position
arm.setPosition(1,100)
arm.setPosition(2,500)
arm.setPosition(3,300)
arm.setPosition(4,680)
arm.setPosition(5,500)
arm.setPosition(6,480)
time.sleep(2)

# 2.90 degree pick-up
arm.setPosition(5,310)
arm.setPosition(4,550)
arm.setPosition(3,600)
time.sleep(2)

# 3.Block Pick-up
arm.setPosition(1,600)
time.sleep(2)

# 4.Back to Set Position
arm.setPosition(2,500)
arm.setPosition(3,300)
arm.setPosition(4,680)
arm.setPosition(5,500)
arm.setPosition(6,500)
time.sleep(2)

# 5.Arm Rotation 90 degree
arm.setPosition(6,850)

# 6.Black Rise on top of the SOARM box
arm.setPosition(5,310)
arm.setPosition(4,550)
arm.setPosition(3,600)
time.sleep(2)
arm.setPosition(1,100)
time.sleep(2)

# 7. Grab the block from the box (Claw close)
arm.setPosition(1,600)
time.sleep(2)

# 8. Lift the block up from the box
arm.setPosition(2,500)
arm.setPosition(3,300)
arm.setPosition(4,680)
arm.setPosition(5,500)
arm.setPosition(6,860) 
time.sleep(2)

# 9. Rotate back to the center (Position 500)
arm.setPosition(6,490)
time.sleep(2)

# 10. Lower down to the original center position
arm.setPosition(5,310)
arm.setPosition(4,550)
arm.setPosition(3,600)
time.sleep(2)

# 11. Open the claw to release the block at home
arm.setPosition(1,100)
time.sleep(2)

# 12. Safe Retract (Lift empty arm up so it doesn't hit the block)
arm.setPosition(2,500)
arm.setPosition(3,300)
arm.setPosition(4,680)
arm.setPosition(5,500)
arm.setPosition(6,500)
time.sleep(2)

print("Done")
