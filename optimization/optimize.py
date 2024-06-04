import json
import os
import sys

from auxiliary_functions import (
    get_params,
    read_binary,
    print_solutions_to_file,
    base_dir,
)

from random_search import RandomSearch

# from bayesian import Bayesian
from genetic import GA
from bayesian import Bayesian
from evaluator import Evaluator
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

base_dir = os.environ["BASE_DIR"]
pd_dir = base_dir + os.getenv("PD_DIR", "/data/PD/")


def optimize(config_file):
    """
    Optimize different trajectories and takes a set of PD controller for each one.
    """

    # general parameters for all the execution(ex tam_population=[10,20,30])
    general_params = get_params(config_file)

    # particular params for 1 alg execution (ex: population size=30)
    params = general_params

    num_joints = len(params["max_torques"])

    # Directory
    controller_name = params["controller_name"]
    # Trajectories to optimize
    path = general_params["path_name"]

    # Speed of the trajectory
    duration_array = params["duration_array"]

    # Trials to compare obtained fronts
    trials = general_params["trials"]

    tam_population = general_params["tam_population"]

    for trial in trials:
        params["trial"] = trial
        for duration in duration_array:
            params["duration"] = duration
            desired_joints = read_binary(
                f"{base_dir}/paths/data/{path}/joints_{duration}"
            )
            desired_joints = desired_joints.reshape(
                len(desired_joints) // num_joints, num_joints
            )

            evaluator = Evaluator(
                general_params["acc_func"],
                general_params["torque_func"],
                desired_joints=desired_joints,
                max_torques=params["max_torques"],
            )

            for tam_p in tam_population:
                params["tam_population"] = tam_p
                # Evaluator construction
                print(
                    f"[{datetime.now()}] Evaluating controller {controller_name} "
                    + f"on trial {trial} with {duration} steps "
                    + f"and {tam_population} of population"
                )

                # Create controller directory
                final_name = f"{controller_name}_{duration}_{tam_p}_{trial}/"
                dir_name = pd_dir + final_name
                os.makedirs(dir_name, exist_ok=True)
                params["experiment_dir"] = dir_name

                with open(dir_name + "/params.json", "w") as f:
                    for key in params:
                        try:
                            params[key] = params[key].tolist()
                        except:
                            pass
                    f.write(str(json.dumps(params, indent=4)))

                params["max_values"] = (
                    np.array(
                        list(params["max_torques"])
                        + [x / 250 for x in params["max_torques"]]
                    )
                    * params["max_value"]
                )  # max_value is usually between 1 and 10

                params[
                    "evaluator"
                ] = evaluator  # evaluator cannot be writen in a json file

                # Optimization Algorithm
                if general_params["algorithm"] == "bayesian":
                    X, F = Bayesian(params, seed=trial)
                elif general_params["algorithm"] == "nsga":
                    X, F = GA(params, seed=trial, algorithm="NSGA")
                elif general_params["algorithm"] == "spea":
                    X, F = GA(params, seed=trial, algorithm="SPEA")
                elif general_params["algorithm"] == "ibea":
                    X, F = GA(params, seed=trial, algorithm="IBEA")
                elif general_params["algorithm"] == "hype":
                    X, F = GA(params, seed=trial, algorithm="HYPE")
                elif general_params["algorithm"] == "moead":
                    X, F = GA(params, seed=trial, algorithm="MOEAD")
                elif general_params["algorithm"] == "random":
                    X, F = RandomSearch(params, seed=trial)

                # Print solutions to an image
                if len(F) > 1:
                    plt.clf()
                    plt.scatter([x[1] for x in F], [x[0] for x in F])
                    plt.title("Objective Space")
                    plt.xlabel("Torque Function")
                    plt.ylabel("Euclidean Error (m)")
                    plt.savefig(dir_name + "pareto_front.png")

                # Save results to files
                print_solutions_to_file(X, F, dir_name, "final")

                params.pop("evaluator")  # evaluator cannot be writen in a json file
                params.pop("max_values")

                print(
                    f"[{datetime.now()}] Finishing evaluation of controller {controller_name} "
                    + f"on trial {trial} with {duration} steps "
                    + f"and {tam_population} of population"
                )


if __name__ == "__main__":
    if len(sys.argv) > 1:
        optimize(sys.argv[1])
    else:
        print("Error: configuration needed")
