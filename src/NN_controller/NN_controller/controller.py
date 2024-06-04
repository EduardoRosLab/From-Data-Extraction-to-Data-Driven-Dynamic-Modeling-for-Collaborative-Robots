import rclpy
from rclpy.node import Node

import numpy as np
import os
import json

from time import time

from custom_msg.msg import Torque, State
from sensor_msgs.msg import JointState

import sys

sys.path.insert(0, "/ML_model")

from BRNN_inv_model import BRNN_model, standarize_data
import tensorflow as tf


class NeuralNetwork(object):
    def __init__(self):
        tf.compat.v1.disable_eager_execution()
        self.model = BRNN_model("/ML_model/brnn_model")
        self.model.load_model()

    def predict_torque(
        self, torque, position, velocity, desired_pos, desired_vel, index
    ):
        if index >= (self.model.params["Tx"] // 2) + 1 and index < (
            len(desired_pos) - (self.model.params["Tx"] // 2)
        ):
            prev_pos = position[-((self.model.params["Tx"] // 2) + 1) :]
            prev_vel = velocity[-((self.model.params["Tx"] // 2) + 1) :]
            future_pos = list(
                desired_pos[index + 1 : 1 + index + (self.model.params["Tx"] // 2)]
            )
            future_vel = list(
                desired_vel[index + 1 : 1 + index + (self.model.params["Tx"] // 2)]
            )

            x = np.array(
                [
                    np.hstack(
                        [
                            np.array(prev_pos + future_pos),
                            np.array(prev_vel + future_vel),
                        ]
                    )
                ]
            )

            x = standarize_data(
                x, self.model.params["x_max"], self.model.params["x_min"]
            )
            tau = self.model.predict_path(x)[0]
        else:
            tau = np.zeros(7)

        return tau


class NN_Controller(Node):
    def __init__(self, params):
        super().__init__("NN_controller")
        self.publisher_ = self.create_publisher(Torque, "torque", 10)
        self.path = params["path"]
        self.max_torques = np.array([320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0])

        #############################################################
        # Initialize model
        self.NN = NeuralNetwork()
        nn_out = self.NN.predict_torque(
            list(np.random.rand(500, 7)),
            list(np.random.rand(500, 7)),
            list(np.random.rand(500, 7)),
            list(np.random.rand(1000, 7)),
            list(np.random.rand(1000, 7)),
            500,
        )
        #############################################################
        # Read PD parameters
        self.kp = np.array(params["kp"])
        self.kd = np.array(params["kd"])
        #############################################################
        # Read trajectory data
        self.trajectory_p = np.fromfile(
            f"/ros_app/paths/data/{self.path}/joints_1000", dtype=np.float64
        )
        self.trajectory_p = self.trajectory_p.reshape(len(self.trajectory_p) // 7, 7)
        self.trajectory_v = np.fromfile(
            f"/ros_app/paths/data/{self.path}/velocity_1000", dtype=np.float64
        )
        self.trajectory_v = self.trajectory_v.reshape(len(self.trajectory_v) // 7, 7)
        #############################################################
        # Initialize data arrays
        self.commanded_torque = []
        self.velocity = []
        self.position = []
        self.pd_torque = []
        self.brnn_torque = []
        self.trajectory_index = -1
        #############################################################
        self.subscription_ = self.create_subscription(
            State, "/custom_joint_states", self.state_received, 10
        )
        self.subscription_

    def state_received(self, msg):
        #############################################################
        # Save data
        self.trajectory_index += 1
        #############################################################
        if self.trajectory_index >= len(self.trajectory_p):
            position = np.array(self.position)
            velocity = np.array(self.velocity)
            commanded_torque = np.array(self.commanded_torque)
            pd_torque = np.array(self.pd_torque)
            brnn_torque = np.array(self.brnn_torque)

            controller_name = "Test"
            dir_name = f"/ros_app/data/NN/{controller_name}/"
            os.makedirs(dir_name, exist_ok=True)
            position.tofile(f"{dir_name}{self.path}_followed")
            velocity.tofile(f"{dir_name}{self.path}_velocity")
            pd_torque.tofile(f"{dir_name}{self.path}_pd")
            brnn_torque.tofile(f"{dir_name}{self.path}_brnn")
            commanded_torque.tofile(f"{dir_name}{self.path}_commanded")

            exit()
        else:
            self.position.append(np.array(msg.position))
            self.velocity.append(np.array(msg.velocity))
            #############################################################
            ### PD
            error_p = self.trajectory_p[self.trajectory_index] - self.position[-1]
            error_v = self.trajectory_v[self.trajectory_index] - self.velocity[-1]
            pd_out = np.array(self.kp * error_p + self.kd * error_v)
            #############################################################
            ### NN Model
            nn_out = self.NN.predict_torque(
                self.commanded_torque,
                self.position,
                self.velocity,
                self.trajectory_p,
                self.trajectory_v,
                self.trajectory_index,
            )

            #############################################################
            msg = Torque()
            if self.trajectory_index <= (
                self.NN.model.params["Tx"] // 2
            ) or self.trajectory_index >= (
                len(self.trajectory_p) - (self.NN.model.params["Tx"] // 2)
            ):
                tau = pd_out
            elif self.trajectory_index <= self.NN.model.params[
                "Tx"
            ] or self.trajectory_index >= (
                len(self.trajectory_p) - self.NN.model.params["Tx"]
            ):
                if self.trajectory_index <= self.NN.model.params["Tx"]:
                    weight = (
                        self.trajectory_index / self.NN.model.params["Tx"]
                    ) * 2 - 1
                else:
                    weight = 1 - (
                        (
                            (
                                self.trajectory_index
                                - (len(self.trajectory_p) - self.NN.model.params["Tx"])
                            )
                            / self.NN.model.params["Tx"]
                        )
                        * 2
                    )

                tau = np.add(pd_out, weight * nn_out)
            else:
                tau = np.add(nn_out, pd_out)
            tau = [
                min(t, mt) if t > 0 else max(t, -mt)
                for t, mt in zip(tau, self.max_torques)
            ]
            msg.torque = tau

            self.publisher_.publish(msg)
            self.commanded_torque.append(tau)
            self.pd_torque.append(pd_out)
            self.brnn_torque.append(nn_out)


def main(args=None):
    rclpy.init(args=args)

    with open("/ML_model/PD/params.json", "r") as f:
        params = json.load(f)

    controller = NN_Controller(params)

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
