#!/usr/bin/env python3

# -*- coding: utf-8 -*-

""" plot.py: Visualises MPC performace from log files."""

__author__ = "Ashwin A Nayar"
__email__ = "ashwin5059198@gmail.com"

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import matplotlib.pyplot as plt
import matplotlib as mpl
import json
import os


class Visualize:
    def __init__(self, data: dict) -> None:
        mpl.rcParams['text.usetex'] = True
        mpl.rcParams['text.latex.preamble'] = r'\usepackage{{amsmath}}'
        plt.style.use('ggplot')

        self.fig = plt.figure("MPC Performance", figsize=(22, 12))
        self.data = data

    def visualize(self) -> None:
        self.vis_trajectory(self.fig.add_subplot(311))
        self.vis_error(self.fig.add_subplot(312))
        self.vis_weights(self.fig.add_subplot(325))
        self.vis_equation(self.fig.add_subplot(326))

    def save(self, path: str) -> None:
        plt.savefig(path)

    def show(self) -> None:
        plt.subplots_adjust(top=0.95, bottom=0.05, left=0.05,
                            right=0.95, hspace=0.28, wspace=0.055)
        plt.get_current_fig_manager().window.showMaximized()
        plt.show()

    def vis_trajectory(self, ax) -> None:
        traj_x = self.data["x"]
        traj_y = self.data["y"]
        costs = self.data["costs"]

        min_cost = min(costs)
        max_cost = max(costs)
        delta = max_cost - min_cost

        for i, cost in enumerate(costs):
            costs[i] = (costs[i] - min_cost) / delta

        ax.set_xlim(-10, 20)
        # ax.set_ylim(-0.05, 1.55)
        ax.plot(traj_x, traj_y, linewidth=1, label="Planned")
        ax.plot(traj_x, [0 for i in range(len(traj_y))],
                linewidth=1, linestyle="dashed", label="Setpoints")
        ax.plot(traj_x, costs, 'k--', linewidth=1, label="Cost distribution")
        ax.fill_between(traj_x, costs, facecolor="#5098d0", alpha=0.5)
        ax.tick_params(axis='both', which='major', labelsize=12)
        ax.legend(prop={'size': 16})
        ax.set_title("Trajectory Information", fontsize=18)

    def vis_error(self, ax) -> None:
        cte = self.data["cte"]
        etheta = self.data["etheta"]
        traj_x = self.data["x"]
        vel = self.data["vel"]

        for i, val in enumerate(cte):
            if val > 20:
                cte[i] = 20
            if val < -20:
                cte[i] = -20

        ax.set_xlim(-10, 20)
        # ax.set_ylim(-1.6, 0.2)
        ax.plot(traj_x, cte, linewidth=1, label="Cross track error")
        ax.plot(traj_x, etheta, linewidth=1, label="Orientation error")
        ax.plot(traj_x, vel, linewidth=1, label="Velocity error")
        ax.tick_params(axis='both', which='major', labelsize=12)
        ax.legend(prop={'size': 16})
        ax.set_title("Error variation", fontsize=18)

    def vis_weights(self, ax) -> None:
        labels = [r'$W_v$', r'$W_d$', r'$W_{\eta}$', r'$W_{\omega}$',
                  r'$W_a$', r'$W_{\dot{\omega}}$', r'$W_{\dot{a}}$']
        weights = [
            self.data["weights"]["vel"],
            self.data["weights"]["cte"],
            self.data["weights"]["etheta"],
            self.data["weights"]["omega"],
            self.data["weights"]["acc"],
            self.data["weights"]["omega_d"],
            self.data["weights"]["acc_d"]
        ]

        for i, w in enumerate(weights):
            weights[i] = round(w, 3)

        for i, val in enumerate(weights):
            ax.text(i, val + 15, str(val), ha='center', size=14)

        ax.set_ylim(0, 100)
        ax.bar(labels, weights, fc=(0.913, 0.639, 0.607, 0.7))
        ax.tick_params(axis='both', which='major', labelsize=16)
        ax.set_title("Weight distribution", fontsize=18)

    def vis_equation(self, ax) -> None:
        equation = r"$J(s) = \sum\limits_{t=0}^N\begin{bmatrix} W_{v} & 0 & 0 \\ 0 & W_{d} & 0 \\ 0 & 0 & W_{\eta} \end{bmatrix}$"
        equation += r"$\begin{bmatrix} \mid\mid{v_{t} - v_{t}^{ref}}\mid\mid^{2} \\ \mid\mid d_{t} \mid\mid^{2} \\ \mid\mid \eta_{t} \mid\mid^{2} \end{bmatrix}$"
        equation += r"$+ \sum\limits_{t=1}^{N-1}\begin{bmatrix} W_{\omega} & 0 \\ 0 & W_{a} \end{bmatrix}$"
        equation += r"$\begin{bmatrix} \mid\mid \omega_{t} \mid\mid^{2} \\ \mid\mid a_{t} \mid\mid^{2} \end{bmatrix}$"
        equation += r"$+ \sum\limits_{t=2}^{N-1}\begin{bmatrix} W_{\dot{\omega}} & 0 \\ 0 & W_{\dot{a}} \end{bmatrix}$"
        equation += r"$\begin{bmatrix} \mid\mid{\omega_{t} - \omega_{t-1}} \mid\mid^{2} \\ \mid\mid{a_{t} - a_{t-1}} \mid\mid^{2} \end{bmatrix}$"
        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_facecolor((1, 1, 1))
        ax.text(0.5, 0.6, equation, ha='center', va='center', size=16)
        ax.set_title("Cost function", fontsize=18)


if __name__ == "__main__":

    data_dir = os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "data")

    if not os.path.exists(os.path.join(data_dir, "plots")):
        os.mkdir(os.path.join(data_dir, "plots"))

    for file in os.listdir(data_dir):
        file_path = os.path.join(data_dir, file)
        if os.path.isfile(file_path):
            data = json.loads(open(file_path).read())
            save_file_name = file.split(".")[0] + ".png"
            save_path = os.path.join(data_dir, "plots", save_file_name)

            if os.path.exists(save_path):
                continue

            vis = Visualize(data)
            vis.visualize()
            vis.save(save_path)
            print("[ Plot-INFO ]: Saved: ", save_file_name)
            plt.clf()
