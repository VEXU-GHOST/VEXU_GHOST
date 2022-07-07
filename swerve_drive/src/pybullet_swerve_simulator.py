import rclpy
import pybullet as p
import time
import pybullet_data
import signal

import sys

def handler(signum, frame):
    print("Exiting")
    p.disconnect()
    sys.exit()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    # p.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1)
    # robotID = p.loadURDF("swerve.urdf")

    step = 1 # ms

    while(True):
        p.stepSimulation()
        time.sleep(step/1000)
