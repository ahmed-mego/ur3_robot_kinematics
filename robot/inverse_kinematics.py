import numpy as np
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH
from robot.dh_model import load_dh_from_yaml  # Your existing function
from zmqRemoteApi import RemoteAPIClient
import time



def compute_inverse_kinematics(config_path, target_pose):


    dh_params = load_dh_from_yaml(config_path)
    links = [RevoluteDH(d=d, a=a, alpha=alpha) for theta, d, a, alpha in dh_params]
    robot = DHRobot(links, name="our robot")

    if isinstance(target_pose, np.ndarray) and target_pose.shape == (4, 4):
        T = SE3(target_pose)
    else:
        raise ValueError("Target pose must be a 4x4 numpy array.")

    result = robot.ikine_LM(T)

    if result.success:
        
        degrees_whole = np.round(np.degrees(result.q))
        return  degrees_whole
    else:
        print("This Position is unreachable")

def send_ur3_joints_to_coppelia(joint_angles_rad):
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # Joint names in your CoppeliaSim scene
    joint_names = [
        '/UR3/joint1',
        '/UR3/joint2',
        '/UR3/joint3',
        '/UR3/joint4',
        '/UR3/joint5',
        '/UR3/joint6'
    ]

    # Get handles to each joint
    joint_handles = [sim.getObject(name) for name in joint_names]

    # Send joint target positions
    for i, angle in enumerate(joint_angles_rad):
        sim.setJointTargetPosition(joint_handles[i], float(angle))

    print("Joint angles sent to UR3.")