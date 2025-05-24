import numpy as np
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH
from robot.dh_model import load_dh_from_yaml  # Your existing function


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
        print("Inverse kinematics failed to converge.")