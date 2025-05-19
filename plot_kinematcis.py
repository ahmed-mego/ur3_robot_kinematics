import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot.dh_model import DHModel, load_dh_from_yaml

def draw_axes(ax, T, name='', length=0.05):
    """Draw coordinate frame at transform T"""
    origin = T[:3, 3]
    x_axis = T[:3, 0] * length
    y_axis = T[:3, 1] * length
    z_axis = T[:3, 2] * length

    ax.quiver(*origin, *x_axis, color='r', label='x' if name == '0' else '')
    ax.quiver(*origin, *y_axis, color='g', label='y' if name == '0' else '')
    ax.quiver(*origin, *z_axis, color='b', label='z' if name == '0' else '')

    ax.text(*origin, f'{name}', fontsize=10)

def plot_robot(dh_model, joint_angles):
    transforms = dh_model.get_joint_transforms(joint_angles)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("UR3 Forward Kinematics Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.grid(True)

    # Plot base frame
    draw_axes(ax, np.eye(4), name='0')

    # Plot all joint frames
    for i, T in enumerate(transforms):
        draw_axes(ax, T, name=str(i + 1))

    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=30, azim=45)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    dh_params = load_dh_from_yaml("~/ur3_robot_kinematics/config/robot_config.yaml")
    model = DHModel(dh_params)

    # Example joint angles (radians)
    joint_angles = [0, -np.pi/4, np.pi/2, 0, np.pi/3, 0]

    plot_robot(model, joint_angles)
