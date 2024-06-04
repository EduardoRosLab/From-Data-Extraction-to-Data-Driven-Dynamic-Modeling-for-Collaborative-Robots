from auxiliary_functions import HV, print_solutions_to_file
from container import simulate

import matplotlib.pyplot as plt
import numpy as np

import os


def evaluate(var, params):
    params[
        "output_filename"
    ] = f"{params['experiment_dir']}/history/{params['path_name']}"
    os.makedirs(
        f"{params['experiment_dir']}/history/",
        exist_ok=True,
    )

    # Cleaning before ending:
    if os.path.isfile(params["output_filename"] + "_torques"):
        os.remove(params["output_filename"] + "_velocity")
        os.remove(params["output_filename"] + "_followed")
        os.remove(params["output_filename"] + "_commanded")
        os.remove(params["output_filename"] + "_torques")

    f1, f2 = simulate(var, params)

    return f1, f2


def update_front(pf_var, pf_func, elem_var, elem_func):
    dominated = []
    for i, f in enumerate(pf_func):
        if elem_func[0] <= f[0] and elem_func[1] <= f[1]:
            dominated.append(i)

    if len(dominated) > 0:
        for i in reversed(dominated):
            del pf_var[i]
            del pf_func[i]

        pf_var.append(elem_var)
        pf_func.append(elem_func)
    else:
        non_dominated = True
        for i, f in enumerate(pf_func):
            if elem_func[0] >= f[0] and elem_func[1] >= f[1]:
                non_dominated = False

        if non_dominated:
            pf_var.append(elem_var)
            pf_func.append(elem_func)

    return pf_var, pf_func


def RandomSearch(params, seed=1):
    ref_point = [0.03, 0.5]
    pf_func = []
    pf_var = []
    params["num_joints"] = len(params["max_torques"])

    rng = np.random.default_rng(seed)

    hv_history = []

    for iteration in range(params["max_evaluations"]):
        params["evaluation"] = iteration
        params["chromosome_id"] = iteration + 10
        var = [
            rng.random() * params["max_value"] * m for m in params["max_torques"]
        ] + [
            (rng.random() * params["max_value"] * m) / 250
            for m in params["max_torques"]
        ]

        func = evaluate(var, params)

        pf_var, pf_func = update_front(pf_var, pf_func, var, func)

        hv = HV(ref_point, pf_func) / (ref_point[0] * ref_point[1])
        hv_history.append(hv)

        np.savetxt(params["experiment_dir"] + "/hv_history.txt", np.array(hv_history))
        plt.title("Hipervolume Indicator")
        plt.plot(np.arange(len(hv_history)), hv_history)
        plt.savefig(params["experiment_dir"] + "/hv_history.png")
        plt.close()

        print_solutions_to_file(pf_var, pf_func, params["experiment_dir"])

    return pf_var, pf_func
