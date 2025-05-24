from robot.forward_kinematics import ForwardKinematics
from robot.dh_model import DHModel, load_dh_from_yaml
import numpy as np
from robot.inverse_kinematics import compute_inverse_kinematics
from robot.jacobian import compute_jacobian 


def main():
    config_path = "C:/Users/Belal Gamal/Documents/ur3_robot_kinematics/config/robot_config.yaml"
   

    fk_solver = ForwardKinematics(config_path)

    # Example joint angles in radians
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
    if ik_degrees is not None:
        # Convert degrees back to radians for FK verification
        ik_radians = np.radians(ik_degrees)
        fk_result = fk_solver.compute(ik_radians)
        verified_pose = fk_result[5]  # End-effector pose after IK → FK

        verified_position = verified_pose[:3, 3]
        print("FK after IK - Verified position (x, y, z):", np.round(verified_position, 3))

        # error check
    error = np.linalg.norm(target_position[:3, 3] - verified_position)
    print("Position error (meters):", np.round(error, 6))

    dh_params = load_dh_from_yaml(config_path)
    jacobian = compute_jacobian(dh_params, joint_angles)
    print("Jacobian matrix:\n", jacobian)

    
if __name__ == "__main__":
    main()
