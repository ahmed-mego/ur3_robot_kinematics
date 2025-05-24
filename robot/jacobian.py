import numpy as np

def compute_dh_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])

def compute_jacobian(dh_params, joint_angles):
    n = len(joint_angles)
    T = np.eye(4)
    origins = [T[:3, 3]]
    z_axes = [T[:3, 2]]

    # Forward kinematics to get all intermediate frames
    for i in range(n):
        theta = joint_angles[i] + dh_params[i][0]  # add offset if needed
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]

        A = compute_dh_matrix(theta, d, a, alpha)
        T = T @ A

        origins.append(T[:3, 3])
        z_axes.append(T[:3, 2])

    o_n = origins[-1]
    J_v = []
    J_w = []

    for i in range(n):
        z = z_axes[i]
        o = origins[i]
        J_v.append(np.cross(z, o_n - o))
        J_w.append(z)

    J_v = np.array(J_v).T
    J_w = np.array(J_w).T

    J = np.vstack((J_v, J_w))
    return np.round(J, 4)