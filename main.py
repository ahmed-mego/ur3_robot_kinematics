from robot.forward_kinematics import ForwardKinematics
from robot.dh_model import DHModel, load_dh_from_yaml
import numpy as np
from robot.inverse_kinematics import compute_inverse_kinematics


def main():
    config_path = "C:/Users/Belal Gamal/Documents/ur3_robot_kinematics/config/robot_config.yaml"
   

    fk_solver = ForwardKinematics(config_path)

    # Example joint angles in radians (UR3 has 6 DOF)
    joint_angles = [0, -np.pi/2, 0, -np.pi/2, 0, 0]

    transforms = fk_solver.compute(joint_angles)

    end_effector_pose = transforms[5]

    position = end_effector_pose[:3, 3]
    
    print("End-effector postion (x, y, z):", np.round(position,3))
    target_position = np.array([
    [1, 0, 0, 0.1],
    [0, 1, 0, 0.3],
    [0, 0, 1, 0.2],
    [0, 0, 0, 1]
    ])
    ik_degrees = compute_inverse_kinematics(config_path, target_position)
    print("IK solution :", ik_degrees)


    # for i, T in enumerate(transforms):
    #     print(f"Transform to joint {i+1}:")
    #     print(np.round(T, 3))
    #     print()

if __name__ == "__main__":
    main()
