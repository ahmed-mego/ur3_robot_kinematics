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

# Connect to CoppeliaSim
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(True)
sim.startSimulation()

# Get joint handles
joint_handles = [sim.getObject(name) for name in joint_names]

# Move each joint to the target angle
for handle, angle in zip(joint_handles, joint_angles):
    sim.setJointTargetPosition(handle, angle)

# Step the simulation for 3 seconds
while (t := sim.getSimulationTime()) < 3:
    print(f"Simulation time: {t:.2f} [s]")
    sim.step()

sim.stopSimulation()
print("Movement complete.")

