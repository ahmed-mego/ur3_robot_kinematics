from robot.dh_model import DHModel

def forward_kinematics(joint_angles, dh_model):
    # Returns end-effector position and full transformation chain
    transforms = dh_model.get_joint_transforms(joint_angles)
    return transforms[-1]  # End-effector pose
print("magdy")
