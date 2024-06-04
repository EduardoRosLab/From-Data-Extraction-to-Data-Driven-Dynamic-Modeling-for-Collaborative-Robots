# %%

from data_generator.kuka_data import get_data
from BRNN_inv_model import BRNN_model
import matplotlib.pyplot as plt
import json
import numpy as np
import sys


def eval(params, verbose=0):
    brnn_inv = BRNN_model(params)
    hyst = brnn_inv.train_model(df, verbose)
    y, y_hat, mae_joint, r2 = brnn_inv.evaluate_path(df_test)
    print("========== Performance \n -MAE_joint: ", mae_joint, "\n -r2: ", r2.numpy())
    return r2.numpy(), brnn_inv


df, df_test = get_data("Fine-Tuned")
model_filename = "/ML_model/torque_model"

params = {
    "n_brnn": 64,
    "n_dense": 32,
    "n_values": 7,
    "Tx": 97,
    "num_features": 14,
    "model_filename": model_filename,
}

r2, model = eval(params, verbose=2)
model.save_model(model_filename)

import tensorflow as tf

tf.compat.v1.disable_eager_execution()

model = BRNN_model(model_filename)
model.load_model()

y, y_hat, _, r2 = model.evaluate_path(df_test)
for joint in range(7):
    plt.plot(y[:, joint], ":", label="Original")
    plt.gca().set_prop_cycle(None)
    plt.plot(y_hat[:, joint], label="Predicted")
    plt.legend()
    plt.savefig(f"/ML_model/Images/test_{joint}.png")
    plt.close()


aux = np.array([np.random.rand(params["Tx"], params["num_features"])])
model.predict_path(aux)

from time import time

comp_time = 0
num_reps = 100
for _ in range(num_reps):
    aux = np.array([np.random.rand(params["Tx"], params["num_features"])])
    begin = time()
    model.predict_path(aux)
    comp_time += time() - begin

print("Computation time:", comp_time / num_reps)
