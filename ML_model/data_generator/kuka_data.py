import numpy as np


def get_data(data_filename):
    #######################################################################
    # Read data files
    paths = ["p0", "p1", "p2", "s0", "s1", "s2", "r0", "r1", "r2"]
    training_paths = paths[:-1]
    test_paths = [paths[-1]]
    torque_files = [
        f"/ML_model/Data/{data_filename}/{path}_commanded" for path in paths
    ]
    position_files = [
        f"/ML_model/Data/{data_filename}/{path}_followed" for path in paths
    ]
    velocity_files = [
        f"/ML_model/Data/{data_filename}/{path}_velocity" for path in paths
    ]
    path_pos = [f"/ros_app/paths/data/{path}/joints_1000" for path in paths]
    path_vel = [f"/ros_app/paths/data/{path}/velocity_1000" for path in paths]
    torque = []
    position = []
    des_p = []
    velocity = []
    des_v = []
    data = {}
    for path, t_filename, p_filename, v_filename, dp_filename, dv_filename in zip(
        paths, torque_files, position_files, velocity_files, path_pos, path_vel
    ):
        torque = np.fromfile(t_filename, dtype=np.float64).reshape(1000, 7)
        position = np.fromfile(p_filename, dtype=np.float64).reshape(1000, 7)
        velocity = np.fromfile(v_filename, dtype=np.float64).reshape(1000, 7)
        des_p = np.fromfile(dp_filename, dtype=np.float64).reshape(1000, 7)
        des_v = np.fromfile(dv_filename, dtype=np.float64).reshape(1000, 7)

        data[path] = np.hstack([position, velocity, torque])

    return [data[p] for p in training_paths], [data[p] for p in test_paths]
