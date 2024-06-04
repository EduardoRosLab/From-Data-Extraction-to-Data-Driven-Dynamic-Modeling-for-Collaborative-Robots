import os
from time import sleep

import numpy as np

from spatialmath.base import q2r
from operator import itemgetter

from robots_models import LBR14820
from custom_msg.msg import Pdparams
from params_node import ControllerOptimizerParamsNode

base_dir = os.environ["BASE_DIR"]


def read_binary(filename):
    return np.fromfile(filename, dtype=np.float64)


def get_non_dominated(population):
    ordered = sorted(population, key=itemgetter(0))
    ret = [ordered[0]]

    for i in range(1, len(ordered)):
        if ordered[i][1] < ret[-1][1]:
            ret.append(ordered[i])

    return ret


def HV(ref_point, population):
    front = get_non_dominated(population)

    acc = 0

    to_delete = []
    for i, f in enumerate(front):
        if f[0] > ref_point[0] or f[1] > ref_point[1]:
            to_delete.append(i)
    for i in reversed(to_delete):
        del front[i]

    if len(front) > 0:
        acc = sum(
            [
                abs(front[i + 1][0] - front[i][0]) * (ref_point[1] - front[i][1])
                for i in range(len(front) - 1)
            ]
        )

        acc += abs(front[-1][0] - ref_point[0]) * (ref_point[1] - front[-1][1])

    return acc


def interpolate_array(arr, int_s):
    return np.array(
        [
            arr[i // int_s]
            + ((i % int_s) / int_s) * (arr[(i // int_s) + 1] - arr[i // int_s])
            for i in range(len(arr) * int_s - (int_s))
        ]
    )


def joints_to_cartesian(joints):
    kuka_model = LBR14820()

    cartesian = np.array([kuka_model.fkine(step).t for step in joints])

    return cartesian


def joints_to_rot(joints):
    kuka_model = LBR14820()

    rotation = np.array([q2r(kuka_model.fkine(step).R) for step in joints])

    return rotation


def print_solutions_to_file(variables, functions, dir_name, base_name):
    os.makedirs(dir_name, exist_ok=True)

    zipped_pairs = zip(functions, variables)

    variables = [x for _, x in sorted(zipped_pairs, key=lambda x: x[0][0])]
    functions = sorted(functions, key=lambda x: x[0])

    np.savetxt(dir_name + base_name + ".VAR", variables)
    np.savetxt(dir_name + base_name + ".FUN", functions)


def get_params(config_file):
    param_node = ControllerOptimizerParamsNode(config_file)

    params = param_node.get_all_params()

    print(f"Controller optimizers parameters loaded: {params}")

    return params
