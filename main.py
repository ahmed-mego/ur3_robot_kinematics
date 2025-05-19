from robot.forward_kinematics import ForwardKinematics
import numpy as np

def main():
    config_path = "~/ur3_robot_kinematics/config/robot_config.yaml"
    fk_solver = ForwardKinematics(config_path)

    # Example joint angles in radians (UR3 has 6 DOF)
    joint_angles = [0, -np.pi/2, 0, -np.pi/2, 0, 0]

    transforms = fk_solver.compute(joint_angles)

    end_effector_pose = transforms[5]

    position = end_effector_pose[:3, 3]

    print("End-effector postion (x, y, z):", np.round(position,3))

    # for i, T in enumerate(transforms):
    #     print(f"Transform to joint {i+1}:")
    #     print(np.round(T, 3))
    #     print()

if __name__ == "__main__":
    main()
