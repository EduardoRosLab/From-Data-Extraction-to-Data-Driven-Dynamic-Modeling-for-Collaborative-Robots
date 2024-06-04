import numpy as np
from auxiliary_functions import joints_to_cartesian, joints_to_rot


class Evaluator:
    def __init__(self, acc_func, torque_func, desired_joints, max_torques):
        acc_functions = {"euclidean": a_euclidean_distance}
        torque_functions = {
            "squared_derivative": t_mean_squared_derivative,
            "mean": t_mean,
        }
        self.acc_func = acc_functions[acc_func]
        self.torque_func = torque_functions[torque_func]
        self.desired_joints = desired_joints
        self.max_torques = max_torques

    def calculate_accuracy(self, followed_path):
        return self.acc_func(followed_path, self.desired_joints)

    def calculate_torque_function(self, torque):
        return self.torque_func(torque, self.max_torques)


# TORQUE FUNCTIONS
def t_mean_squared_derivative(torque, max_torques):
    joints_torque = [np.array([x[i] for x in torque]) for i in range(len(max_torques))]
    joint_val = np.power(
        [
            [joint[i] - joint[i - 1] for i in range(1, len(joint))]
            for joint in joints_torque
        ],
        2,
    )

    return np.mean(joint_val)


def t_mean(torque, max_torques):
    joints_torque = [np.array([x[i] for x in torque]) for i in range(len(max_torques))]
    mean_torque = [
        np.mean(np.abs(jt)) / mt for jt, mt in zip(joints_torque, max_torques)
    ]

    return np.mean(mean_torque)


# ACCURACY FUNCTIONS
## Average of the euclidean distance.
## Orientation error ignored.
def a_euclidean_distance(path, desired_joints):
    followed_cartesian = joints_to_cartesian(path)
    desired_cartesian = joints_to_cartesian(desired_joints)

    distances = np.power(desired_cartesian - followed_cartesian, 2)

    error = np.array([np.sqrt(d[0] + d[1] + d[2]) for d in distances])

    return np.mean(error)


## Euclidean distance + orientation error
def a_euclidean_orientation(path, desired_joints):
    # Euclidean distance
    followed_cartesian = joints_to_cartesian(path)
    desired_cartesian = joints_to_cartesian(desired_joints)

    distances = np.power(desired_cartesian - followed_cartesian, 2)

    cartesian_error = np.mean(
        np.array([np.sqrt(d[0] + d[1] + d[2]) for d in distances])
    )

    # Orientation error
    followed_rotation = joints_to_rot(path)
    desired_rotation = joints_to_rot(desired_joints)

    rotation_error = np.mean(
        [np.abs(np.dot(f, d)) for f, d in zip(followed_rotation, desired_rotation)]
    )

    return cartesian_error * rotation_error  # Geometric mean (without sqrt)
