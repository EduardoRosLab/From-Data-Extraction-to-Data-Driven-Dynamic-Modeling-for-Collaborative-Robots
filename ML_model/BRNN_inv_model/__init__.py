import numpy as np
import tensorflow as tf
import utils as ut
import json
from numba import jit

# physical_devices = tf.config.list_physical_devices('GPU')
# tf.config.experimental.set_memory_growth(physical_devices[0], enable=True)


def destandarize_data(data, maxi, mini):
    return np.array([v * (maxi - mini) + mini for v in data])


def standarize_data(data, maxi, mini):
    return np.array([(v - mini) / (maxi - mini) for v in data])


# %%
class BRNN_model(object):

    def __init__(self, params):
        if type(params) is str:
            with open(params + ".json", "r") as f:
                self.params = json.load(f)
            self.params["x_max"] = np.array(self.params["x_max"])
            self.params["y_max"] = np.array(self.params["y_max"])
            self.params["x_min"] = np.array(self.params["x_min"])
            self.params["y_min"] = np.array(self.params["y_min"])
        else:
            self.params = params
        self.brnn_model = self.new_model()

    def new_model(
        self,
    ):
        rnn_for = tf.keras.layers.GRU(
            self.params["n_brnn"],
            return_sequences=False,
            return_state=False,
            name="rnn_for",
        )
        rnn_back = tf.keras.layers.GRU(
            self.params["n_brnn"],
            return_sequences=False,
            return_state=False,
            go_backwards=True,
            name="rnn_back",
        )
        densor = tf.keras.layers.Dense(units=self.params["n_values"], name="densor_out")

        x = tf.keras.Input(shape=(self.params["Tx"], self.params["num_features"]))
        x_norm = tf.keras.layers.LayerNormalization()(x)
        int_mid = int(self.params["Tx"] / 2)

        x_start2mid = tf.keras.layers.Lambda(lambda z: z[:, : (int_mid + 1), :])(x)
        x_mid2end = tf.keras.layers.Lambda(lambda z: z[:, int_mid:, :])(x)

        a1 = rnn_for(inputs=x_start2mid)
        a2 = rnn_back(inputs=x_mid2end)
        a = tf.concat([a1, a2], axis=-1)
        out = tf.keras.layers.Dense(units=self.params["n_values"], name="densor_out")(a)

        brnn_model = tf.keras.Model(inputs=[x], outputs=out)

        brnn_model.compile(
            loss=tf.losses.MeanSquaredError(),
            optimizer=tf.optimizers.Adam(learning_rate=0.001),
        )

        return brnn_model

    def train_model(self, df, verbose=0):
        df_data = []
        for df_i in df:
            df_data.append(df_i)

        step = 1
        x_train, y_train, _ = sort_samples_inv(
            df_data[:-1],
            self.params["Tx"],
            step,
            num_features=self.params["num_features"],
        )
        x_val, y_val, _ = sort_samples_inv(
            [df_data[-1]],
            self.params["Tx"],
            step,
            num_features=self.params["num_features"],
        )

        max_torques = np.array([320.0, 320.0, 176.0, 176.0, 110.0, 40.0, 40.0])
        self.params["y_max"] = max_torques
        self.params["y_min"] = -max_torques

        print(x_train.shape)
        limits = np.array(
            [2.96706, 2.094395, 2.96706, 2.094395, 2.96706, 2.094395, 3.054326]
            + [3] * 7
        )
        self.params["x_max"] = limits
        self.params["x_min"] = -limits

        y_train = standarize_data(y_train, self.params["y_max"], self.params["y_min"])
        y_val = standarize_data(y_val, self.params["y_max"], self.params["y_min"])

        x_train = standarize_data(x_train, self.params["x_max"], self.params["x_min"])
        x_val = standarize_data(x_val, self.params["x_max"], self.params["x_min"])

        print("===== Model training....")

        early_stop = tf.keras.callbacks.EarlyStopping(
            monitor="val_loss",
            min_delta=0,
            patience=100,
            restore_best_weights=True,
            start_from_epoch=500,
        )

        history = self.brnn_model.fit(
            x_train,
            y_train,
            validation_data=(x_val, y_val),
            epochs=5000,
            batch_size=128,
            verbose=verbose,
            callbacks=[early_stop],
        )
        print("===== Training end!!")
        return history

    def predict_path(self, q_qd_path):
        data = self.brnn_model.predict(q_qd_path, verbose=0, batch_size=1)
        return destandarize_data(
            data,
            self.params["y_max"],
            self.params["y_min"],
        )

    def evaluate_path(self, data_path):
        df_data = []
        for df_i in data_path:
            df_data.append(df_i)
        x, y, _ = sort_samples_inv(
            df_data, self.params["Tx"], step=1, num_features=self.params["num_features"]
        )
        x = standarize_data(x, self.params["x_max"], self.params["x_min"])
        y_hat = self.predict_path(x)

        mae_joint, r2 = calc_performance(y, y_hat)

        return y, y_hat, mae_joint, r2

    def save_model(self, filename):
        self.brnn_model.save_weights(filename)
        self.params["x_max"] = list(self.params["x_max"])
        self.params["y_max"] = list(self.params["y_max"])
        self.params["x_min"] = list(self.params["x_min"])
        self.params["y_min"] = list(self.params["y_min"])
        with open(filename + ".json", "w") as f:
            json.dump(self.params, f)

    def load_model(self):
        self.brnn_model.load_weights(self.params["model_filename"]).expect_partial()


# %%
def sort_samples_inv(df, maxlen, step, num_features):
    x_q_qd = []
    y_tau = []
    long_traj = []

    for traj in df:
        x, y = ut.samples_sort(traj, maxlen, step, num_features=num_features)
        x_q_qd.append(x)
        y_tau.append(y)
        long_traj.append(x.shape[0])

    x_q_qd = np.concatenate(x_q_qd, axis=0)
    y_tau = np.concatenate(y_tau, axis=0)

    return x_q_qd, y_tau, long_traj


def calc_performance(y, y_hat):
    r2 = R_squared(y, y_hat)
    mae_joint = np.mean(np.abs(y - y_hat), axis=0)

    return mae_joint, r2


def R_squared(y, y_pred):
    residual = tf.reduce_sum(tf.square(tf.subtract(y, y_pred)))
    total = tf.reduce_sum(tf.square(tf.subtract(y, tf.reduce_mean(y))))
    r2 = tf.subtract(
        tf.cast(1.0, tf.float64), tf.cast(tf.divide(residual, total), tf.float64)
    )

    return r2
