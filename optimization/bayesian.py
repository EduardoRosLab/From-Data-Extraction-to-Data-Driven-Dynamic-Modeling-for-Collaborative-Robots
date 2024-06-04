from container import simulate
from bayes_opt import BayesianOptimization
import numpy as np
import os
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
from bayes_opt.util import load_logs
import matplotlib.pyplot as plt

from os.path import exists

from glob import glob

base_dir = os.environ["BASE_DIR"]
pd_dir = base_dir + os.getenv("PD_DIR", "/data/PD/")


def Objective_Function(x, params):
    params[
        "output_filename"
    ] = f"{params['experiment_dir']}/history/{params['evaluation']}/{params['path_name']}"
    os.makedirs(
        f"{params['experiment_dir']}/history/{params['evaluation']}/",
        exist_ok=True,
    )
    os.chmod(
        f"{params['experiment_dir']}/history/{params['evaluation']}/",
        0o777,
    )

    # Cleaning before starting:
    if os.path.isfile(params["output_filename"] + "_torques"):
        os.remove(params["output_filename"] + "_velocity")
        os.remove(params["output_filename"] + "_followed")
        os.remove(params["output_filename"] + "_commanded")
        os.remove(params["output_filename"] + "_torques")

    f1, f2 = simulate(x, params)

    with open(params["experiment_dir"] + "/history.txt", "a") as f:
        f.write(str(f1) + "\n")

    with open(params["experiment_dir"] + "/history.txt", "r") as f:
        lines = f.readlines()
        archive = [float(x) for x in lines]

    plt.plot(np.arange(len(archive)), archive)
    plt.title("Accuracy Function")
    plt.savefig(params["experiment_dir"] + "/history.png")
    plt.close()

    with open(params["experiment_dir"] + "/best_history.txt", "a") as f:
        f.write(str(min(archive)) + "\n")

    with open(params["experiment_dir"] + "/best_history.txt", "r") as f:
        lines = f.readlines()
        best_archive = [float(x) for x in lines]

    plt.plot(np.arange(len(best_archive)), best_archive)
    plt.title("Best Controller Found")
    plt.savefig(params["experiment_dir"] + "/best_history.png")
    plt.close()

    params["evaluation"] += 1

    return f1, f2


def Bayesian(params, seed=1):
    params["chromosome_id"] = 2
    params["num_joints"] = len(params["max_torques"])
    try:
        params["evaluation"] = max(
            [
                int(x.split("/")[-1])
                for x in glob(f"{params['experiment_dir']}/history/*")
            ]
        )
    except ValueError:
        params["evaluation"] = 0

    OF = lambda p1, p2, p3, p4, p5, p6, p7, d1, d2, d3, d4, d5, d6, d7: -Objective_Function(
        np.array([p1, p2, p3, p4, p5, p6, p7, d1, d2, d3, d4, d5, d6, d7]),
        params,
    )[
        0
    ]

    pb = {}
    for i, var in enumerate(
        [
            "p1",
            "p2",
            "p3",
            "p4",
            "p5",
            "p6",
            "p7",
            "d1",
            "d2",
            "d3",
            "d4",
            "d5",
            "d6",
            "d7",
        ]
    ):
        pb[var] = (0, params["max_values"][i])

    optimizer = BayesianOptimization(
        f=OF,
        pbounds=pb,
        random_state=seed,
    )

    optimizer.set_gp_params(alpha=1e-3, n_restarts_optimizer=0)

    if exists(f"{params['experiment_dir']}/logs.json"):
        load_logs(optimizer, logs=[f"{params['experiment_dir']}/logs.json"])
        params["tam_population"] = 0

    logger = JSONLogger(path=f"{params['experiment_dir']}/logs.json", reset=False)
    optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    optimizer.maximize(
        init_points=params["tam_population"],
        n_iter=params["max_evaluations"],
    )

    results = optimizer.max
    print("Result: {}".format(results))

    constants = results["params"]

    return (
        np.array(
            [
                [
                    constants["p1"],
                    constants["p2"],
                    constants["p3"],
                    constants["p4"],
                    constants["p5"],
                    constants["p6"],
                    constants["p7"],
                    constants["d1"],
                    constants["d2"],
                    constants["d3"],
                    constants["d4"],
                    constants["d5"],
                    constants["d6"],
                    constants["d7"],
                ]
            ]
        ),
        [[results["target"]]],
    )
