from robot.dh_model import DHModel
import numpy as np

def compute_jacobian(dh_params, joint_angles):
    model = DHModel(dh_params)
    transforms = model.get_joint_transforms(joint_angles)

    origins = [np.array([0, 0, 0])]
    z_axes = [np.array([0, 0, 1])]

    for T in transforms:
        origins.append(T[:3, 3])
        z_axes.append(T[:3, 2])

    o_n = origins[-1]
    J_v = []
    J_w = []

    for i in range(len(joint_angles)):
        z = z_axes[i]
        o = origins[i]
        J_v.append(np.cross(z, o_n - o))
        J_w.append(z)

    J = np.vstack((np.array(J_v).T, np.array(J_w).T))
    return np.round(J, 4)