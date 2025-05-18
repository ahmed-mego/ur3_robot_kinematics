from robot.dh_model import DHModel, load_dh_from_yaml
from robot.forward_kinematics import forward_kinematics
import numpy as np

def main():
    # Load UR3 DH parameters
    dh_params = load_dh_from_yaml("~/projects/ur3_robot_kinematics/config/robot_config.yaml")
    print(dh_params)
    # dh_model = DHModel(dh_params)

    # # Example joint angles in radians [θ1 to θ6]
    # joint_angles = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]

    # # Compute forward kinematics
    # ee_pose = forward_kinematics(joint_angles, dh_model)

    # print("UR3 End-effector pose (4x4 homogeneous transform):\n", ee_pose)

if __name__ == "__main__":
    main()
