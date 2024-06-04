## Params Node

import json


class ControllerOptimizerParamsNode:
    """
    Param node for the controller optimizer.
    Accepts the following parameters:
    - path_name -> str: path to be given to the controller.
    - max_value -> int: maximum value for the controller parameters
    - max_evaluations -> int: maximum number of evaluations for the genetic algorithm
    - tam_population -> list(int): list of the population sizes for the genetic algorithm
    - controller_name -> str: name of the controller to be optimized
    - duration_array -> list(int): duration of each trajectory (in nº of control loops)
    - distribution_index -> distribution index for combination and mutation operators
    - algorithm -> str: algorithm to use in the optimization (bayesian, NSGA)
    - trials -> list(int): trial seeds
    - controller_indexes -> list(int): indexes of the controllers to use in "save" executable
    - acc_func -> str: accuracy function to use during the optimization
    - torque_func -> str: torque function to use during the optimization
    - dataset_name -> string: name of the dataset to be used in the "save" function
    - max_torques -> list(int): torque limits on each joint
    - joint_limits -> list(float): position limits on each joint
    - joint_names -> list(string): name of each joint
    - optimization_seeds -> list(string): names of controllers to use as seeds for optimization
    """

    def __init__(self, config_file) -> None:
        self.default = dict(
            [
                ("path_name", "spiral"),
                ("max_value", 5),
                ("max_evaluations", 5000),
                ("tam_population", [30]),
                ("controller_name", "S0_HiL_Deriv_8"),
                ("duration_array", [1000]),
                ("distribution_index", 20),
                ("algorithm", "NSGA"),
                ("trials", [0]),
                ("controller_indexes", [-1]),
                ("acc_func", "euclidean"),
                ("torque_func", "squared_derivative"),
                ("dataset_name", "test_local"),
                ("max_torques", [320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0]),
                (
                    "joint_limits",
                    [2.96706, 2.094395, 2.96706, 2.094395, 2.96706, 2.094395, 3.054326],
                ),
                (
                    "joint_names",
                    [
                        "joint_a1",
                        "joint_a2",
                        "joint_a3",
                        "joint_a4",
                        "joint_a5",
                        "joint_a6",
                        "joint_a7",
                    ],
                ),
                ("optimization_seeds", ["None"]),
            ]
        )

        try:
            with open(config_file, "r") as f:
                file_dict = json.load(f)
        except FileNotFoundError:
            file_dict = eval(config_file)

        self.params = dict(
            [(key, file_dict.get(key, self.default[key])) for key in self.default]
        )

    def get_all_params(self) -> dict:
        return self.params

    def pprint_params(self) -> None:
        pprint(self.get_all_params())

    @property
    def get_parameters_name(self):
        return self.default.keys()
