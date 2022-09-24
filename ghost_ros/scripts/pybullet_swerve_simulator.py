import os
import sys
import time
import signal
from collections import OrderedDict

import rclpy
import pybullet as p
import numpy as np
from ament_index_python.packages import get_package_share_directory

swerve_mod_src_path = os.path.join(get_package_share_directory("ghost_ros"), "scripts")
sys.path.append(swerve_mod_src_path)

from pybullet_rviz_publisher import PybulletRVIZPublisher

def handler(signum, frame):
    print("Exiting")
    p.disconnect()
    sys.exit()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    
    # Initialize PyBullet Server
    physicsClient = p.connect(p.DIRECT)
    
    # Set Simulation Parameters
    p.setGravity(0, 0, -9.81)
    # p.setPhysicsEngineParameter(fixedTimeStep=DT, numSubSteps=1)
    step = 1 # ms
    
    # Load Robot and Ground models
    urdf_dir = os.path.join(get_package_share_directory("ghost_ros"), "urdf")
    p.loadURDF(urdf_dir + "/plane.urdf", [0, 0, 0], useFixedBase=True)
    robotID = p.loadURDF(
        urdf_dir + "/swerve.urdf", 
        basePosition=[0.0, 0.0, 1.0],
        baseOrientation=[0.0, 0.0, 0.0, 1.0],
        flags=p.URDF_USE_INERTIA_FROM_FILE)

    num_joints = p.getNumJoints(robotID)
    print("Number of Joints: " + str(num_joints))

    # Initialize Joint States from Simulator
    joint_states = [None] * num_joints

    for i in range(num_joints):
        joint_states[i] = OrderedDict()
        joint_data = p.getJointState(robotID, i)
        
        joint_states[i]["name"] = p.getJointInfo(robotID, i)[1].decode("ascii")
        joint_states[i]["jointPosition"] = joint_data[0]
        joint_states[i]["jointVelocity"] = joint_data[1]

    # Initialize Wheel Contacts Dynamics
    p.changeDynamics(robotID, 2, lateralFriction = 1.0, spinningFriction = 1.0, rollingFriction = 0.0,
                     restitution = 0.0, contactStiffness = 20000, contactDamping = 10000)

    p.changeDynamics(robotID, 4, lateralFriction = 1.0, spinningFriction = 1.0, rollingFriction = 0.0,
                     restitution = 0.0, contactStiffness = 20000, contactDamping = 10000)

    p.changeDynamics(robotID, 6, lateralFriction = 1.0, spinningFriction = 1.0, rollingFriction = 0.0,
                     restitution = 0.0, contactStiffness = 20000, contactDamping = 10000)

    # Instantiate RVIZ frontend
    rviz_publisher = PybulletRVIZPublisher()

    # Run Simulation Loop
    while(True):
        p.stepSimulation()

        # Position and Orientation of base with respect to world
        base_position = p.getBasePositionAndOrientation(robotID)[0]
        base_orientation = p.getBasePositionAndOrientation(robotID)[1]

        # Linear and Angular velocity in world frame
        base_linear_velocity = p.getBaseVelocity(robotID)[0]
        base_angular_velocity = p.getBaseVelocity(robotID)[1]

        # Transform to robot base frame from world frame
        rotate_vector_to_robot = np.linalg.inv(np.array(p.getMatrixFromQuaternion(base_orientation)).reshape(3, 3))
        robot_linear_velocity = rotate_vector_to_robot @ base_linear_velocity
        robot_angular_velocity = base_angular_velocity

        # Update Joint States
        for i in range(num_joints):
            joint_data = p.getJointState(robotID, i)
            joint_states[i]["jointPosition"] = joint_data[0]
            joint_states[i]["jointVelocity"] = joint_data[1]

        rviz_publisher.update_base_position(base_position, base_orientation)
        rviz_publisher.update_base_velocity(robot_linear_velocity, robot_angular_velocity)
        rviz_publisher.update_joint_states(joint_states)

        p.setJointMotorControlArray(
            robotID,
            list(range(num_joints)),
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.5
                ]
        )

        # contact = p.getContactPoints()
        # for i in contact:
        #     print(i)
        # print()
        # print()
        # print()

        dyn = p.getDynamicsInfo(robotID, 2)
        print("Mass:", dyn[0])
        print("Lateral Friction:", dyn[1])
        print("Restitution:", dyn[5])
        print("Rolling Friction:", dyn[6])
        print("Spinning Friction:", dyn[7])
        print("Contact Damping:", dyn[8])
        print("Contact Stiffness:", dyn[9])
        print()
        print()

        time.sleep(step/1000)
