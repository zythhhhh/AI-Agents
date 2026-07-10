import xarm
import time

# Initialize connections to all three USB ports
arm1 = xarm.Controller("USB0")  # Furthest right arm
arm2 = xarm.Controller("USB1")  # Far left arm
arm3 = xarm.Controller("USB2")  # Middle-right arm

# Calibrated claw positions
CLAW_OPEN = 100
CLAW_CLOSE = 700

print("Initializing all arms to YOUR custom initial state...")

# =====================================================================
# YOUR EXACT INITIAL STATE
# =====================================================================
arm1.setPosition(2, 900)
arm1.setPosition(3, 300)
arm1.setPosition(4, 680)
arm1.setPosition(5, 500)
arm1.setPosition(6, 500)

arm2.setPosition(2, 850)
arm2.setPosition(3, 500)
arm2.setPosition(4, 500)
arm2.setPosition(5, 500)
arm2.setPosition(6, 500)

arm3.setPosition(2, 500)
arm3.setPosition(3, 500)
arm3.setPosition(4, 500)
arm3.setPosition(5, 500)
arm3.setPosition(6, 500)

# Wait 2 seconds for all arms to stabilize in their initial positions
time.sleep(2)

# =====================================================================
# MOVEMENT 1: Open Claws and Execute Synchronized Multi-Arm Move
# =====================================================================
print("Opening claws and executing simultaneous movements across all 3 arms...")

# --- ARM 1: Open claw, rotate 90° right, and bend down ---
arm1.setPosition(1, CLAW_OPEN)  # Open Arm 1 Claw
arm1.setPosition(6, 100)        # Rotate Right
arm1.setPosition(4, 300)        # Bend Down (Decreasing ticks)

# --- ARM 2: Open claw and bend 90° down (Opposite Dir) ---
arm2.setPosition(1, CLAW_OPEN)  # Open Arm 2 Claw
arm2.setPosition(4, 850)        # Bend Down (Increasing ticks)

# --- ARM 3: Open claw and bend 90° down (Standard Dir) ---
arm3.setPosition(1, CLAW_OPEN)  # Open Arm 3 Claw
arm3.setPosition(4, 850)        # Bend Down (Increasing ticks)

# Wait 2 seconds for all arms to fully reach their targets
time.sleep(2)

# =====================================================================
# MOVEMENT 2: Synchronized Claw Closure
# =====================================================================
print("Target reached. Closing all 3 claws simultaneously...")

arm1.setPosition(1, CLAW_CLOSE)  # Close Arm 1 Claw
arm2.setPosition(1, CLAW_CLOSE)  # Close Arm 2 Claw
arm3.setPosition(1, CLAW_CLOSE)  # Close Arm 3 Claw

# Give the grippers 2 seconds to firmly secure the objects
time.sleep(2)

arm1.setPosition(1, CLAW_OPEN)  # Close Arm 1 Claw
arm2.setPosition(1, CLAW_OPEN)  # Close Arm 2 Claw
arm3.setPosition(1, CLAW_OPEN)  # Close Arm 3 Claw
time.sleep(2)

arm1.setPosition(1, CLAW_CLOSE)  # Close Arm 1 Claw
arm2.setPosition(1, CLAW_CLOSE)  # Close Arm 2 Claw
arm3.setPosition(1, CLAW_CLOSE)  # Close Arm 3 Claw

print("Done.")