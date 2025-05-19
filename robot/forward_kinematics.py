from robot.dh_model import DHModel, load_dh_from_yaml

class ForwardKinematics:
    def __init__(self, dh_config_path):
        self.dh_params = load_dh_from_yaml(dh_config_path)
        self.model = DHModel(self.dh_params)

    def compute(self, joint_angles):
        return self.model.get_joint_transforms(joint_angles)
