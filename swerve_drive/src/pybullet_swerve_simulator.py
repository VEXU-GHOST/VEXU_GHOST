import rclpy
import pybullet as p
import time
import pybullet_data
import signal

import os
from ament_index_python.packages import get_package_share_directory

import sys

def handler(signum, frame):
    print("Exiting")
    p.disconnect()
    sys.exit()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    
    # Initialize PyBullet Server
    physicsClient = p.connect(p.GUI)
    
    # Set Simulation Parameters
    p.setGravity(0, 0, -9.81)
    # p.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1)
    step = 1 # ms
    
    # Load Robot and Ground models
    urdf_dir = os.path.join(get_package_share_directory("swerve_drive"), "urdf")
    # p.loadURDF(urdf_dir + "/plane.urdf", [0, 0, 0])
    robotID = p.loadURDF(urdf_dir + "/swerve.urdf", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

    num_joints = p.getNumJoints(robotID)
    print("Number of Joints: " + str(num_joints))

    # Run Simulation Loop
    while(True):
        p.stepSimulation()
        # print(p.getBasePositionAndOrientation(robotID))
        for i in range(num_joints):
            print("Joint ID:", i)
            print(p.getJointInfo(robotID, i)[1])
            print(p.getJointState(robotID, i))
        # print(p.getBasePositionAndOrientation(robotID))
        time.sleep(step/1000)
