<<<<<<< HEAD
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
=======
# from robot.forward_kinematics import ForwardKinematics
# import numpy as np

# def main():
#     config_path = "~/ur3_robot_kinematics/config/robot_config.yaml"
#     fk_solver = ForwardKinematics(config_path)

#     # Example joint angles in radians (UR3 has 6 DOF)
#     joint_angles = [0, -np.pi/2, 0, -np.pi/2, 0, 0]

#     transforms = fk_solver.compute(joint_angles)

#     end_effector_pose = transforms[5]

#     position = end_effector_pose[:3, 3]

#     print("End-effector postion (x, y, z):", np.round(position,3))

#     # for i, T in enumerate(transforms):
#     #     print(f"Transform to joint {i+1}:")
#     #     print(np.round(T, 3))
#     #     print()

# if __name__ == "__main__":
#     main()


# from robot.forward_kinematics import ForwardKinematics
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# import numpy as np
# import time

# def main():
#     # 1. Load configuration and FK
#     config_path = "~/ur3_robot_kinematics/config/robot_config.yaml"
#     fk_solver = ForwardKinematics(config_path)

#     # Joint angles in radians
#     joint_angles = [np.pi/2, -np.pi/2, 0, -np.pi/2, 0, 0]

#     # 2. Compute Forward Kinematics
#     transforms = fk_solver.compute(joint_angles)
#     end_effector_pose = transforms[-1]

#     print("End-effector pose:")
#     print(np.round(end_effector_pose, 3))

#     # 3. Connect to CoppeliaSim
#     client = RemoteAPIClient()
#     sim = client.require('sim')
#     sim.setStepping(True)
#     sim.startSimulation()

#     # 4. Joint names and handles (use the ones you discovered)
#     joint_names = [
#         '/UR3/joint',
#         '/UR3/link/joint',
#         '/UR3/link/joint/link/joint',
#         '/UR3/link/joint/link/joint/link/joint',
#         '/UR3/link/joint/link/joint/link/joint/link/joint',
#         '/UR3/joint/link/joint/link/joint/link/joint/link/joint/link/joint'
#     ]
#     joint_handles = [sim.getObject(name) for name in joint_names]

#     # 5. Move robot by setting joint target positions
#     for i, handle in enumerate(joint_handles):
#         sim.setJointTargetPosition(handle, joint_angles[i])

#     # 6. Step simulation for a few seconds
#     while (t := sim.getSimulationTime()) < 30:
#         print(f"Simulation time: {t:.2f} [s]")
#         sim.step()

#     sim.stopSimulation()
#     print("Simulation complete.")

# if __name__ == "__main__":
#     main()


import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

# Define joint angles (in radians)
# These should match the ones you used for forward kinematics
joint_angles = [0, 0, 0, -np.pi/2, 0, 0]

# Define joint names based on your UR3 robot in CoppeliaSim
joint_names = [
    '/UR3/joint',
    '/UR3/link/joint',
    '/UR3/link/joint/link/joint',
    '/UR3/link/joint/link/joint/link/joint',
    '/UR3/link/joint/link/joint/link/joint/link/joint',
    '/UR3/joint/link/joint/link/joint/link/joint/link/joint/link/joint'
]
>>>>>>> e6d4b05eb29e9bed93863bd20ed87398f5a5eaa9

# Connect to CoppeliaSim
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(True)
sim.startSimulation()

# Get joint handles
joint_handles = [sim.getObject(name) for name in joint_names]

<<<<<<< HEAD
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
<<<<<<< HEAD
    main()
=======
# Move each joint to the target angle
for handle, angle in zip(joint_handles, joint_angles):
    sim.setJointTargetPosition(handle, angle)

# Step the simulation for 3 seconds
while (t := sim.getSimulationTime()) < 3:
    print(f"Simulation time: {t:.2f} [s]")
    sim.step()

sim.stopSimulation()
print("Movement complete.")

>>>>>>> e6d4b05eb29e9bed93863bd20ed87398f5a5eaa9
=======
    main()
>>>>>>> 8facd499a9c0b1e6c50f9abbcb156c1b631e45ec
