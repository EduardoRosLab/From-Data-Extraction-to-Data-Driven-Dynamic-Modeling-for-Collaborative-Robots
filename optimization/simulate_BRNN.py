import sys
import os
import json
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

from custom_msg.msg import Pdparams
from evaluator import Evaluator

from auxiliary_functions import (
    read_binary,
    base_dir,
    HV,
    print_solutions_to_file,
    joints_to_cartesian,
)


from jmetal.algorithm.multiobjective.nsgaii import NSGAII
from jmetal.operator import SBXCrossover, PolynomialMutation
from jmetal.core.problem import FloatProblem
from jmetal.core.solution import FloatSolution
from jmetal.util.termination_criterion import TerminationCriterion
from jmetal.util.solution import get_non_dominated_solutions


def simulate(x, path, ev_id=0):

    max_torques = np.array([320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0])
    num_joints = 7

    kp = list(x[:7])
    kd = list(x[7:])
    params = {"path": path, "kp": kp, "kd": kd}
    with open(f"{base_dir}/ML_model/PD/params.json", "w") as f:
        json.dump(params, f)

    duration = 1000

    data_dir = base_dir + "/data/NN/Test/"
    sim_data_dir = "/ros_app/data/NN/Test/"
    output_filename = f"{data_dir}{path}"
    sim_output_filename = f"{sim_data_dir}{path}"

    # Load joint trajectory
    desired_joints = read_binary(f"{base_dir}/paths/data/{path}/joints_{duration}")
    desired_joints = desired_joints.reshape(
        len(desired_joints) // num_joints, num_joints
    )

    desired_velocity = read_binary(f"{base_dir}/paths/data/{path}/velocity_{duration}")
    desired_velocity = desired_velocity.reshape(
        len(desired_velocity) // num_joints, num_joints
    )

    evaluator = Evaluator(
        "euclidean",
        "squared_derivative",
        desired_joints=desired_joints,
        max_torques=max_torques,
    )

    # Execute trajectory
    while not os.path.isfile(output_filename + "_commanded"):
        os.system(
            f'docker compose run --rm ros sh -c "cp /ros_app/paths/data/{path}/initial_positions.yaml /ros_app/initial_positions.yaml && ros2 launch bringup nn.launch.py > /dev/null 2>&1  &  sleep 1 && ros2 launch bringup external_controller.launch.py > /dev/null 2>&1 &  /ML_model/wait.sh {sim_output_filename}"'
        )

    followed_path = read_binary(output_filename + "_followed")
    followed_path = followed_path.reshape(len(followed_path) // 7, 7)

    velocity = read_binary(output_filename + "_velocity")
    velocity = velocity.reshape(len(velocity) // 7, 7)

    torque = read_binary(output_filename + "_commanded")
    torque = torque.reshape(len(followed_path), 7)

    f_accuracy = evaluator.calculate_accuracy(followed_path)
    f_torque = evaluator.calculate_torque_function(torque)
    print(
        "\n{} - Accuracy Function: {} Torque Function: {}".format(
            path, f_accuracy, f_torque
        )
    )

    ### SAVE IMAGES

    cartesian = joints_to_cartesian(followed_path)
    desired_cartesian = joints_to_cartesian(desired_joints)

    fig = plt.figure()
    ax = plt.axes(projection="3d")

    ax.plot3D(cartesian[:, 0], cartesian[:, 1], cartesian[:, 2], label="Followed")
    ax.plot3D(
        desired_cartesian[:, 0],
        desired_cartesian[:, 1],
        desired_cartesian[:, 2],
        label="Original",
    )
    plt.legend()
    plt.savefig(output_filename + f"_cartesian_{ev_id}.png")
    plt.close()

    for i in range(7):
        ## Positions
        plt.plot(
            np.arange(len(followed_path)),
            followed_path[:, i],
            c=[0, 0, 1],
            label="Followed",
        )
        plt.plot(
            np.arange(len(desired_joints)),
            desired_joints[:, i],
            c=[1, 0, 1],
            label="Original",
        )
        plt.legend()

        plt.savefig(output_filename + f"_positions_{i}_{ev_id}.png")
        plt.close()

        ## Torque
        plt.plot(
            np.arange(len(torque)),
            np.transpose(torque)[i, :],
            c=[0, 0, 1],
        )

        plt.savefig(output_filename + f"_torque_{i}_{ev_id}.png")
        plt.close()

        ## Velocity
        plt.plot(
            np.arange(len(velocity)),
            velocity[:, i],
            c=[0, 0, 1],
            label="Followed",
        )
        plt.plot(
            np.arange(len(desired_velocity)),
            desired_velocity[:, i],
            c=[1, 0, 1],
            label="Original",
        )
        plt.legend()

        plt.savefig(output_filename + f"_velocity_{i}_{ev_id}.png")
        plt.close()

    os.remove(output_filename + "_commanded")
    os.remove(output_filename + "_pd")
    os.remove(output_filename + "_brnn")
    os.remove(output_filename + "_followed")
    os.remove(output_filename + "_velocity")

    return f_accuracy, f_torque


class Controller_Optimization(FloatProblem):
    def __init__(self):
        super(Controller_Optimization, self).__init__()
        self.name = "name"
        self.n_of_variables: int = 2 * 7
        self.n_of_objectives: int = 2
        self.n_of_constraints: int = 0

        self.reference_front: List[S] = None

        self.directions: List[int] = []
        self.labels: List[str] = []

        self.lower_bound = np.zeros(14)
        max_torques = np.array([320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0])
        self.upper_bound = 5 * np.concatenate([max_torques, max_torques / 250])
        self.ev_id = 0

    def number_of_objectives(self) -> int:
        return self.n_of_objectives

    def number_of_variables(self) -> int:
        return self.n_of_variables

    def number_of_constraints(self) -> int:
        return self.n_of_constraints

    def name(self):
        return "name"

    def evaluate(self, solution: FloatSolution) -> FloatSolution:

        sum_acc = 0
        sum_torque = 0
        paths = ["p0", "p1", "p2", "s0", "s1", "s2", "r0", "r1", "r2"]
        for path in paths:
            acc, torque = solution.objectives = simulate(
                solution.variables, path, self.ev_id
            )
            sum_acc += acc
            sum_torque = torque

        self.ev_id += 1
        solution.objectives = [sum_acc / len(paths), sum_torque / len(paths)]

        return solution


class StoppingByHV(TerminationCriterion):
    def __init__(
        self,
        population_size,
        max_eval,
        ref_point=[0.03, 0.5],
        offspring_size=1,
    ):
        super().__init__()
        self.max_eval = max_eval
        self.population_size = population_size
        self.num_eval = 0
        self.ref_point = ref_point
        self.hv_history = []
        self.offspring_size = offspring_size

    def update(self, *args, **kwargs):
        solutions = kwargs["SOLUTIONS"]
        population = np.array([x.variables for x in solutions])
        F = np.array([x.objectives for x in solutions])
        self.num_eval += 1

        ### SAVE DATA

        self.hv_history.append(
            HV(self.ref_point, F) / (self.ref_point[0] * self.ref_point[1])
        )

        print_solutions_to_file(population, F, f"{base_dir}/ML_model/PD/", "population")

        plt.figure(0)
        np.savetxt(f"{base_dir}/ML_model/PD/hv_history.txt", np.array(self.hv_history))
        plt.title("Hipervolume Indicator")
        plt.plot(np.arange(self.num_eval), self.hv_history)
        plt.savefig(f"{base_dir}/ML_model/PD/hv_history.png")
        plt.close()

        info = "Nº eval: {}".format(self.num_eval)
        print(info)

    @property
    def is_met(self):
        if len(self.hv_history) < (5 * self.population_size // self.offspring_size):
            return False
        return (self.num_eval >= self.max_eval) or (
            0.9995
            < self.hv_history[-(5 * self.population_size) // self.offspring_size]
            / self.hv_history[-1]
        )


params = {
    "tam_population": 60,
    "max_evaluations": 4000,
}
problem = Controller_Optimization()

dt = 20
crossover = SBXCrossover(probability=1.0, distribution_index=dt)
mutation = PolynomialMutation(
    probability=1.0 / problem.number_of_variables(), distribution_index=dt
)
term_crit = StoppingByHV(
    params["tam_population"],
    params["max_evaluations"],
)

algorithm = NSGAII(
    problem=problem,
    population_size=params["tam_population"],
    offspring_population_size=1,
    mutation=mutation,
    crossover=crossover,
    termination_criterion=term_crit,
)

algorithm.run()
