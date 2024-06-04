from auxiliary_functions import HV, print_solutions_to_file
from container import simulate

import matplotlib.pyplot as plt
import numpy as np

import os

from glob import glob

from jmetal.algorithm.multiobjective.spea2 import SPEA2
from jmetal.algorithm.multiobjective.nsgaii import NSGAII
from jmetal.algorithm.multiobjective.ibea import IBEA
from jmetal.algorithm.multiobjective.hype import HYPE
from jmetal.algorithm.multiobjective.moead import MOEAD
from jmetal.util.aggregative_function import Tschebycheff
from jmetal.operator import SBXCrossover, PolynomialMutation
from jmetal.core.problem import FloatProblem
from jmetal.core.solution import FloatSolution
from jmetal.util.termination_criterion import TerminationCriterion
from jmetal.util.solution import get_non_dominated_solutions
from jmetal.util.aggregative_function import Tschebycheff

from datetime import datetime


base_dir = os.environ["BASE_DIR"]
pd_dir = base_dir + os.getenv("PD_DIR", "/data/PD/")

# Jmetal


class Controller_Optimization(FloatProblem):
    def __init__(self, params):
        super(Controller_Optimization, self).__init__()
        self.params = params
        self.params["num_joints"] = len(self.params["max_torques"])
        self.name = "name"
        self.n_of_variables: int = 2 * params["num_joints"]
        self.n_of_objectives: int = 2
        self.n_of_constraints: int = 0

        self.reference_front: List[S] = None

        self.directions: List[int] = []
        self.labels: List[str] = []

        self.lower_bound = np.zeros(2 * params["num_joints"])
        self.upper_bound = params["max_values"]

        self.params["chromosome_id"] = 2
        # try:
        #     self.params["evaluation"] = max(
        #         [
        #             int(x.split("/")[-1])
        #             for x in glob(f"{params['experiment_dir']}/history/*")
        #         ]
        #     )
        # except ValueError:
        self.params["evaluation"] = 0

    def number_of_objectives(self) -> int:
        return self.n_of_objectives

    def number_of_variables(self) -> int:
        return self.n_of_variables

    def number_of_constraints(self) -> int:
        return self.n_of_constraints

    def name(self):
        return "name"

    def evaluate(self, solution: FloatSolution) -> FloatSolution:
        self.params[
            "output_filename"
        ] = f"{self.params['experiment_dir']}/history/{self.params['path_name']}"
        os.makedirs(
            f"{self.params['experiment_dir']}/history/",
            exist_ok=True,
        )

        # Cleaning before ending:
        if os.path.isfile(self.params["output_filename"] + "_torques"):
            os.remove(self.params["output_filename"] + "_velocity")
            os.remove(self.params["output_filename"] + "_followed")
            os.remove(self.params["output_filename"] + "_commanded")
            os.remove(self.params["output_filename"] + "_torques")

        solution.objectives = simulate(solution.variables, self.params)

        self.params["evaluation"] += 1

        return solution


class StoppingByHV(TerminationCriterion):
    def __init__(
        self,
        population_size,
        max_eval,
        experiment_dir,
        ref_point=[0.03, 0.5],
        offspring_size=1,
    ):
        super().__init__()
        self.experiment_dir = experiment_dir
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

        print_solutions_to_file(population, F, experiment_dir, "population")

        plt.figure(0)
        np.savetxt(self.experiment_dir + "/hv_history.txt", np.array(self.hv_history))
        plt.title("Hipervolume Indicator")
        plt.plot(np.arange(self.num_eval), self.hv_history)
        plt.savefig(self.experiment_dir + "/hv_history.png")
        plt.close()

        # plt.figure(1)
        # np.savetxt(
        #     self.experiment_dir
        #     + "/history/"
        #     + str(self.num_eval - 2 + len(population))
        #     + "/population.txt",
        #     np.array(F),
        # )
        # plt.title("Current Population")
        # plt.scatter(np.array(F[:, 1]), np.array(F[:, 0]))
        # plt.xlim(0, self.ref_point[1])
        # plt.xlabel("Torque Function")
        # plt.ylim(0, self.ref_point[0])
        # plt.ylabel("Accuracy Function")
        # plt.savefig(
        #     self.experiment_dir
        #     + "/history/"
        #     + str(self.num_eval - 2 + len(population))
        #     + "/population.png"
        # )
        # plt.close()

        ### ACTUAL TERMINATION CRITERION
        info = "[{}] Nº eval: {}".format(datetime.now(), self.num_eval)
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


def GA(params, seed=1, algorithm="NSGA"):
    ref_point = [0.03, 0.5]

    # Directory
    dir_name = params["experiment_dir"]

    # Problem optimization
    problem = Controller_Optimization(
        params,
    )

    dt = params["distribution_index"]
    crossover = SBXCrossover(probability=1.0, distribution_index=dt)
    mutation = PolynomialMutation(
        probability=1.0 / problem.number_of_variables(), distribution_index=dt
    )
    term_crit = StoppingByHV(
        params["tam_population"],
        params["max_evaluations"],
        params["experiment_dir"],
        ref_point=ref_point,
    )
    if algorithm == "NSGA":
        algorithm = NSGAII(
            problem=problem,
            population_size=params["tam_population"],
            offspring_population_size=1,
            mutation=mutation,
            crossover=crossover,
            termination_criterion=term_crit,
        )
    elif algorithm == "SPEA":
        algorithm = SPEA2(
            problem=problem,
            population_size=params["tam_population"],
            offspring_population_size=1,
            mutation=mutation,
            crossover=crossover,
            termination_criterion=term_crit,
        )
    elif algorithm == "HYPE":
        hype_ref = FloatSolution(
            [0],
            [1],
            problem.number_of_objectives(),
        )
        hype_ref.objectives = ref_point
        algorithm = HYPE(
            problem=problem,
            population_size=params["tam_population"],
            reference_point=hype_ref,
            offspring_population_size=1,
            mutation=mutation,
            crossover=crossover,
            termination_criterion=term_crit,
        )
    elif algorithm == "MOEAD":
        off_size = 10
        algorithm = MOEAD(
            problem=problem,
            population_size=params["tam_population"],
            crossover=crossover,
            mutation=mutation,
            aggregative_function=Tschebycheff(dimension=problem.number_of_objectives()),
            neighbor_size=off_size,
            neighbourhood_selection_probability=0.9,
            max_number_of_replaced_solutions=2,
            weight_files_path="/tmp/MOEAD_weights",
            termination_criterion=StoppingByHV(
                params["tam_population"],
                params["max_evaluations"],
                params["experiment_dir"],
                ref_point=ref_point,
                offspring_size=off_size,
            ),
        )
    elif algorithm == "IBEA":
        algorithm = IBEA(
            problem=problem,
            kappa=1.0,
            population_size=params["tam_population"],
            offspring_population_size=1,
            mutation=mutation,
            crossover=crossover,
            termination_criterion=term_crit,
        )

    algorithm.run()
    solutions = algorithm.get_result()
    front = get_non_dominated_solutions(solutions)

    return [s.variables for s in front], [s.objectives for s in front]
