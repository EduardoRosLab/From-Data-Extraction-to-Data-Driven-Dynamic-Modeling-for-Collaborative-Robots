from pymoo.core.termination import Termination
from datetime import datetime
from collections import deque
import numpy as np

import matplotlib.pyplot as plt

from pymoo.indicators.hv import HV


class StoppingByNonDominated(Termination):
    def __init__(
        self, population_size, max_eval, experiment_dir, ref_point=[0.03, 0.5]
    ):
        super().__init__()
        self.experiment_dir = experiment_dir
        self.max_eval = max_eval
        self.population_size = population_size
        self.archive = deque(maxlen=5 * population_size)
        self.counter = 0
        self.num_eval = 0
        self.ref_point = ref_point
        self.p_ind = HV(ref_point)
        self.hv_history = []

    def _update(self, algorithm):
        if self.num_eval >= self.max_eval:
            return 1.0

        population = algorithm.pop
        self.num_eval += 1

        ### SAVE DATA

        F = np.array([q.F for q in population])
        self.hv_history.append(self.p_ind(F) / (self.ref_point[0] * self.ref_point[1]))

        with open(f"{self.experiment_dir}/population.VAR", "w") as vf:
            with open(f"{self.experiment_dir}/population.FUN", "w") as ff:
                for p in population:
                    for x in p.X:
                        vf.write(str(x) + " ")
                    for f in p.F:
                        ff.write(str(f) + " ")
                    vf.write("\n")
                    ff.write("\n")

        plt.figure(0)
        np.savetxt(self.experiment_dir + "/hv_history.txt", np.array(self.hv_history))
        plt.title("Hipervolume Indicator")
        plt.plot(np.arange(self.num_eval), self.hv_history)
        plt.savefig(self.experiment_dir + "/hv_history.png")
        plt.close()

        plt.figure(1)
        np.savetxt(
            self.experiment_dir
            + "/history/"
            + str(self.num_eval - 2 + len(population))
            + "/population.txt",
            np.array(F),
        )
        plt.title("Current Population")
        plt.scatter(F[:, 1], F[:, 0])
        plt.xlim(0, self.ref_point[1])
        plt.xlabel("Torque Function")
        plt.ylim(0, self.ref_point[0])
        plt.ylabel("Accuracy Function")
        plt.savefig(
            self.experiment_dir
            + "/history/"
            + str(self.num_eval - 2 + len(population))
            + "/population.png"
        )
        plt.close()

        ### ACTUAL TERMINATION CRITERION

        if len(self.hv_history) > 5 * len(population):
            ret = self.hv_history[-(5 * len(population))] / self.hv_history[-1] + 0.0005
        else:
            ret = 0.0

        info = "[{}] Nº eval: {} Progress: {}".format(
            datetime.now(), self.num_eval, ret
        )
        print(info)

        return ret

        # front_len = 0
        # for p in population:
        #     dominated = False
        #     for q in population:
        #         if all(np.less(q.F, p.F)):
        #             dominated = True
        #             break
        #     if not dominated:
        #         front_len += 1

        # ratio = front_len / len(population)

        # self.archive.append(ratio)

        # info = "[{}] Nº eval: {} Mean: {}  STD: {}  Ratio: {}".format(
        #     datetime.now(),
        #     self.num_eval,
        #     round(np.mean(self.archive), 3),
        #     round(np.std(self.archive), 3),
        #     round(ratio, 2),
        # )

        # print(info)

        # if len(self.archive) == 5 * self.population_size:
        #     if np.mean(self.archive) > 0.95 and np.std(self.archive) < 0.025:
        #         return 1.0

        # return np.mean(self.archive)
