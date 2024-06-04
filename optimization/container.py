import sys
import os
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

from custom_msg.msg import Pdparams

from auxiliary_functions import read_binary, joints_to_cartesian


# 'docker compose run --rm ros sh -c "python3 /optimization/optimize.py /optimization/experiments/spiral/spiral.json"'


def simulate(x, params):
    start_command = "ros2 launch bringup simulation.launch.py &"

    stop_command = "killall ros2 && killall -9 gzserver && killall -w robot_state_pub"

    got_results = False
    while not got_results:
        os.system(
            f"cp /ros_app/paths/data/{params['path_name']}/initial_positions.yaml /ros_app/initial_positions.yaml"
        )
        p = os.system(start_command)

        # ------------------------
        params["kp"] = x[: params["num_joints"]]
        params["kd"] = x[params["num_joints"] :]
        # ------------------------

        msg = {}

        msg["id"] = params["chromosome_id"]
        msg["evaluation"] = params["evaluation"]
        msg["path"] = params["path_name"]
        msg["kp"] = list(params["kp"])
        msg["kd"] = list(params["kd"])
        msg[
            "joints_filename"
        ] = f"/ros_app/paths/data/{params['path_name']}/joints_{params['duration']}"
        msg[
            "velocity_filename"
        ] = f"/ros_app/paths/data/{params['path_name']}/velocity_{params['duration']}"

        msg["output_filename"] = params["output_filename"]
        msg["joint_names"] = params["joint_names"]
        msg["max_torques"] = params["max_torques"]
        msg["joint_limits"] = params["joint_limits"]

        os.system(
            'ros2 topic pub -t 2 /params custom_msg/msg/Pdparams "{}" &'.format(msg)
        )

        time_waiting = 0
        time_step = 0.2

        while not os.path.isfile(params["output_filename"] + "_commanded"):
            if time_waiting > 40:
                break
            sleep(time_step)
            time_waiting += time_step

        os.system(stop_command)

        if os.path.isfile(params["output_filename"] + "_commanded"):
            got_results = True
            followed_path = read_binary(params["output_filename"] + "_followed")
            followed_path = followed_path.reshape(len(followed_path) // 7, 7)

            torque = read_binary(params["output_filename"] + "_commanded")
            torque = torque.reshape(len(followed_path), 7)

            f_accuracy = params["evaluator"].calculate_accuracy(followed_path)
            f_torque = params["evaluator"].calculate_torque_function(torque)
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
            plt.savefig(params["output_filename"] + "_positions.png")
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
            plt.savefig(params["output_filename"] + "_torque.png")
            plt.close()

    return round(f_accuracy, 4), round(f_torque, 4)  # Rounding
