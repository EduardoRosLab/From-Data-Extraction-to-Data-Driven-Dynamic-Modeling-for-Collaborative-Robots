import sys
import os
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

from custom_msg.msg import Pdparams
from evaluator import Evaluator

from auxiliary_functions import read_binary, joints_to_cartesian, base_dir


start_command = "ros2 launch bringup simulation.launch.py &"

stop_command = "killall ros2 && killall -9 gzserver && killall -w robot_state_pub"

max_torques = [320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0]
num_joints = 7

paths = ["p0", "p1", "p2", "s0", "s1", "s2", "r0", "r1", "r2"]
indexes = [21, 25, 3, 3, 37, 6, 24, 7, 29]
duration = 1000

pd_dir = base_dir + "/data/PD/"
data_dir = base_dir + "/data/datasets/"

for i, path, index in zip(np.arange(len(paths)), paths, indexes):
    output_filename = f"{data_dir}test/{path}"

    # Load controller
    controllers = np.loadtxt(f"{pd_dir}nsga_{path}_100_1000_60_0/population.VAR")
    # controllers = np.loadtxt(f"{pd_dir}general_nsga_1000_60_0/population.VAR")
    x = controllers[index]

    # Load joint trajectory
    desired_joints = read_binary(f"{base_dir}/paths/data/{path}/joints_{duration}")
    desired_joints = desired_joints.reshape(
        len(desired_joints) // num_joints, num_joints
    )

    evaluator = Evaluator(
        "euclidean",
        "squared_derivative",
        desired_joints=desired_joints,
        max_torques=max_torques,
    )

    # Execute trajectory
    got_results = False
    while not got_results:
        os.system(
            f"cp /ros_app/paths/data/{path}/initial_positions.yaml /ros_app/initial_positions.yaml"
        )
        p = os.system(start_command)

        msg = {}

        msg["id"] = i + 5
        msg["evaluation"] = i + 5
        msg["path"] = path
        msg["kp"] = list(x[:num_joints])
        msg["kd"] = list(x[num_joints:])
        msg["joints_filename"] = f"/ros_app/paths/data/{path}/joints_{duration}"
        msg["velocity_filename"] = f"/ros_app/paths/data/{path}/velocity_{duration}"

        msg["output_filename"] = output_filename
        msg["joint_names"] = [
            "joint_a1",
            "joint_a2",
            "joint_a3",
            "joint_a4",
            "joint_a5",
            "joint_a6",
            "joint_a7",
        ]
        msg["max_torques"] = max_torques
        msg["joint_limits"] = [
            2.96706,
            2.094395,
            2.96706,
            2.094395,
            2.96706,
            2.094395,
            3.054326,
        ]

        os.system(
            'ros2 topic pub -t 2 /params custom_msg/msg/Pdparams "{}" &'.format(msg)
        )

        time_waiting = 0
        time_step = 0.2

        while not os.path.isfile(output_filename + "_commanded"):
            if time_waiting > 40:
                break
            sleep(time_step)
            time_waiting += time_step

        os.system(stop_command)

        if os.path.isfile(output_filename + "_commanded"):
            got_results = True
            followed_path = read_binary(output_filename + "_followed")
            followed_path = followed_path.reshape(len(followed_path) // 7, 7)

            torque = read_binary(output_filename + "_commanded")
            torque = torque.reshape(len(followed_path), 7)

            f_accuracy = evaluator.calculate_accuracy(followed_path)
            f_torque = evaluator.calculate_torque_function(torque)
            print(
                "Accuracy Function: {}\nTorque Function: {}".format(
                    f_accuracy, f_torque
                )
            )

            ### SAVE IMAGES
            desired_path = read_binary(msg["joints_filename"])
            desired_path = desired_path.reshape(len(followed_path), 7)

            ## Positions
            fig, ax = plt.subplots(7, figsize=(8, 10))
            for i in range(7):
                ax[i].plot(
                    np.arange(len(followed_path)),
                    followed_path[:, i],
                    c=[0, 0, 1],
                    label="Followed",
                )
                ax[i].plot(
                    np.arange(len(desired_path)),
                    desired_path[:, i],
                    c=[1, 0, 1],
                    label="Original",
                )
                ax[i].set_title("Articulación {}".format(i))
                ax[i].tick_params(labelbottom=False, direction="in")

            fig.text(
                0.01,
                0.5,
                "Error Joints",
                va="center",
                rotation="vertical",
            )
            ax[6].tick_params(labelbottom=True)
            ax[6].set_title("Articulación {}".format(6))
            ax[6].set_xlabel("Instante de tiempo (2ms)")
            ax[6].legend()

            plt.tight_layout()
            plt.savefig(output_filename + "_positions.png")
            plt.close()

            ## Torque
            figure, axis = plt.subplots(7, figsize=(8, 10))
            for i in range(7):
                axis[i].plot(
                    np.arange(len(torque)),
                    np.transpose(torque)[i],
                    c=[0, 0, 1],
                )
                axis[i].set_title("Articulación {}".format(i))
                axis[i].tick_params(labelbottom=False, direction="in")

            figure.text(
                0.01,
                0.5,
                "Par motor (Nm)",
                va="center",
                rotation="vertical",
            )
            axis[6].tick_params(labelbottom=True)
            axis[6].set_title("Articulación {}".format(6))
            axis[6].set_xlabel("Instante de tiempo (2ms)")

            plt.tight_layout()
            plt.savefig(output_filename + "_torque.png")
            plt.close()
