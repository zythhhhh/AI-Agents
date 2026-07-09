import xarm
import time

arm1 = xarm.Controller("USB0")
arm2 = xarm.Controller("USB1")
arm3 = xarm.Controller("USB2")

arm1.setPosition(3, 300)
arm2.setPosition(3, 300)
arm3.setPosition(3, 300)


