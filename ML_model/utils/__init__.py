import numpy as np
from scipy import signal

lim_max = np.array(
    [1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094, 3.059]
    + [2] * 4
    + [4] * 3
    + [50] * 4
    + [15] * 3
)
lim_min = np.array(
    [-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059]
    + [-2] * 4
    + [-4] * 3
    + [-50] * 4
    + [-15] * 3
)
train_mean = lim_min
train_std = lim_max - lim_min


def norm_max_min_pos_vel(data_orig):
    data_norm = (data_orig - train_mean[:14]) / train_std[:14]

    return data_norm


def norm_max_min(data_orig):
    data_norm = (data_orig - train_mean) / train_std

    return data_norm


def norm_max_min_torq(data_orig):
    data_norm = (
        data_orig - np.concatenate((train_mean, train_mean[14:21]))
    ) / np.concatenate((train_std, train_std[14:21]))

    return data_norm


def norm_max_min_sim_torq(data_orig):
    data_norm = (data_orig - train_mean[14:21]) / train_std[14:21]

    return data_norm


def dnorm_max_min(data_norm):
    data_org = data_norm * train_std[14:21] + train_mean[14:21]

    return data_org


def dnorm_max_min_pos_vel(data_norm):
    data_org = data_norm * train_std[:14] + train_mean[:14]

    return data_org


def samples_sort(
    samples, size_wind, step=1, output=True, offset=25, num_features=14, num_values=7
):
    offset = size_wind
    x_samples = []
    y_label = []
    ini_wind = offset - int(size_wind / 2)

    for i in range(0, len(samples) - 2 * offset, step):
        x_samples.append(
            samples[i + ini_wind : i + ini_wind + size_wind, 0:num_features]
        )
        if output:
            y_label.append(
                samples[i + offset, num_features : num_features + num_values]
            )
        else:
            y_label.append([0] * num_values)
    print("nb sequences:", len(x_samples))

    x = np.array(x_samples)
    y = np.array(y_label)

    return x, y


def samples_sort_torq(samples, size_wind, step=1, output=True, offset=25):
    x_samples = []
    y_label = []
    ini_wind = offset - int(size_wind / 2)

    for i in range(0, len(samples) - 2 * offset, step):
        x_samples.append(samples[i + ini_wind : i + ini_wind + size_wind, 0:21])
        if output:
            y_label.append(samples[i + offset, 21:28])
        else:
            y_label.append([0] * 7)
    print("nb sequences:", len(x_samples))

    x = np.array(x_samples)
    y = np.array(y_label)

    return x, y


def sort_samples_forward(samples, size_wind, step, output=True, offset=25):
    x_samples = []
    y_label = []
    init_wind = offset
    final_wind = len(samples) - size_wind - offset

    for i in range(init_wind, final_wind, step):
        x_samples.append(samples[i : i + size_wind, :])
        if output:
            y_label.append(samples[i + 1 : i + size_wind + 1, :14])
        else:
            y_label.append([0] * 14)
    print("nb sequences:", len(x_samples))

    x = np.array(x_samples)
    y = np.array(y_label)

    return x, y


def sort_samples(df, maxlen, step):

    x_pos_vel_torq = []
    y_pos_vel_real = []
    long_trajct = []

    for tray in df:
        x, y = sort_samples_forward(tray, maxlen, step=step, offset=0)
        x_pos_vel_torq.append(x)
        y_pos_vel_real.append(y)
        long_trajct.append(x.shape[0])

    x_pos_vel_torq = np.concatenate(x_pos_vel_torq, axis=0)
    y_pos_vel_real = np.concatenate(y_pos_vel_real, axis=0)

    return x_pos_vel_torq, y_pos_vel_real, long_trajct


def idx_split_traj(size_data2split, ratio_val=0.20, num_split=3):
    mask_idx_all = []

    for i, long_traj in enumerate(size_data2split):
        mask_idx = np.zeros((num_split, long_traj))
        idx_cut = np.random.randint(0, long_traj, size=(num_split))
        long_div = int(long_traj * ratio_val)
        idx_end = idx_cut + long_div

        for n_split, (id_c, id_e) in enumerate(zip(idx_cut, idx_end)):
            print(n_split, id_c, id_e)
            if id_e >= long_traj:
                mask_idx[n_split, id_c:] = True
                mask_idx[n_split, : id_e - long_traj] = True
            else:
                mask_idx[n_split, id_c:id_e] = True

        mask_idx = np.bool_(mask_idx)
        mask_idx_all.append(mask_idx)

    mask_idx_all = np.concatenate(mask_idx_all, axis=-1)

    return mask_idx_all


def get_train_val(x, y, mask_idx):
    x_val = x[mask_idx]
    y_val = y[mask_idx]
    x_train = x[np.logical_not(mask_idx)]
    y_train = y[np.logical_not(mask_idx)]

    return x_train, y_train, x_val, y_val


def normalization(data, mean, std):
    return (data - mean) / std


def d_normalization(data, mean, std):
    return (data * std) + mean


def load_dataset(path_data, name_files, delta, factor=1):
    delta_t = delta * factor

    df = [
        np.genfromtxt(path_data + nfile, skip_header=1, delimiter=",")[::factor, 1:22]
        for nfile in name_files
    ]

    x = []

    for traj in df:
        pos_tmp, vel_tmp, acl_tmp = preprocess_traj(traj[:, :7], delta_t)
        data_tmp = np.concatenate([pos_tmp, vel_tmp, acl_tmp, traj[:, 14:21]], axis=-1)
        x.append(data_tmp)

    return x


def get_train_val_vector(data_x, data_y, long_traj, val=0.2):
    mask_idx = idx_split_traj(long_traj, num_split=1, ratio_val=val)

    return get_train_val(data_x, data_y, mask_idx[0])


# %%
def preprocess_traj(traj, delta_t):
    sos = signal.butter(3, 5, fs=1.0 / delta_t, output="sos")

    pos_filt = signal.sosfiltfilt(sos, traj[:, :7], axis=0)
    vel_est = calculate_drv(pos_filt, delta_t)
    acl_est = calculate_drv(vel_est, delta_t)

    return pos_filt, vel_est, acl_est


def calculate_drv(q_vel, delta_t):
    q_acl = np.zeros(q_vel.shape)

    q_acl[0] = (q_vel[1] - q_vel[0]) / delta_t

    q_acl[1:-1] = (q_vel[2:] - q_vel[:-2]) / (2 * delta_t)

    q_acl[-1] = (q_vel[-1] - q_vel[-2]) / delta_t

    return q_acl


# %%
def samples_ord_inv_v2(samples, size_wind, step=1, output=True, offset=25):
    x_samples = []
    y_label = []
    ini_wind = offset - int(size_wind / 2)

    for idx_ini in range(ini_wind, len(samples) - 2 * offset, step):
        x_samples.append(samples[idx_ini : idx_ini + size_wind, :21])
        if output:
            y_label.append(samples[idx_ini : idx_ini + size_wind, 21:])
        else:
            y_label.append([0] * 7)
    print("nb sequences:", len(x_samples))

    x = np.array(x_samples)
    y = np.array(y_label)

    return x, y


def ordenar_samples_v2(df, maxlen, step):

    x_pos_vel_torq = []
    y_pos_vel_real = []
    long_trajct = []

    for tray in df:
        x, y = samples_ord_inv_v2(tray, maxlen, step=step)
        x_pos_vel_torq.append(x)
        y_pos_vel_real.append(y)
        long_trajct.append(x.shape[0])

    x_pos_vel_torq = np.concatenate(x_pos_vel_torq, axis=0)
    y_pos_vel_real = np.concatenate(y_pos_vel_real, axis=0)

    return x_pos_vel_torq, y_pos_vel_real, long_trajct
