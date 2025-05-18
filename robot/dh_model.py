import numpy as np
import os
import yaml

class DHModel:
    def __init__(self, dh_params):
        self.dh_params = dh_params  # List of [theta_offset, d, a, alpha]

    def compute_transformation_matrix(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,      ca,      d],
            [0,        0,       0,      1]
        ])

    def get_joint_transforms(self, joint_angles):
        assert len(joint_angles) == len(self.dh_params), "Mismatch between joint angles and DH parameters"

        transforms = []
        current_T = np.eye(4)
        
        for i, (theta_offset, d, a, alpha) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            T = self.compute_transformation_matrix(theta, d, a, alpha)
            current_T = current_T @ T
            transforms.append(current_T)
        
        return transforms

def load_dh_from_yaml(path):
    full_path = os.path.expanduser(path)
    with open(full_path, 'r') as f:
        data = yaml.safe_load(f)
    return data["robot"]["dh_parameters"]