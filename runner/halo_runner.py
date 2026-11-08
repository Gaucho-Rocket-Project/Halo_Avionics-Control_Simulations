"""Closed-loop runner. One accepted integrator step, then the controller.

The BDF solver still takes its own internal substeps inside `solver.step()`.
This loop does not call the controller on those substeps. The command returned
here is stored on the plant and is used on the next step.

BDF : backward differentiation formula, it is the numerical method this simulation uses
to integrate the rockets equations of motion forward in time.
"""

import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import BDF


class Runner:
    def __init__(self, sim, controls):
        self.sim = sim
        self.controls = controls
        self.clear_cache()

    def solve(self, State0=[0, 0, 0,
                            0, 0, 0,
                            0, 0, 0,
                            0, 0, 0, 285]):
        initial_command = [0, 0, 0, 0]
        self.controls_cache.append([initial_command])
        self.sim.command = initial_command
        control_times = [0]

        # Kalman state is initialized from the true initial state.
        kalman_states_x = [np.array([State0[i] for i in [0, 2, 3, 5, 7, 10]]) * np.array([1, 1, 1, 1, -1, -1])]
        kalman_states_y = [np.array([State0[i] for i in [1, 2, 4, 5, 6, 9]]) * np.array([-1, 1, -1, 1, -1, -1])]
        kalman_roll = [[0, 0]]
        kalman_cov_x = [np.zeros((6, 6))]
        kalman_cov_y = [np.zeros((6, 6))]
        self.control_states_cache.append([[kalman_states_x[-1], kalman_states_y[-1], kalman_roll[-1], kalman_cov_x[-1], kalman_cov_y[-1]]])

        timestamps = []
        states = []
        status = 0
        start_time = time.time()
        solver = BDF(self.sim.update, 0, State0, self.sim.params.t_final)
        while solver.status == "running":
            solver.step()

            w_x = np.random.multivariate_normal(np.zeros(6), self.sim.params.Q_kalman)
            w_y = np.random.multivariate_normal(np.zeros(6), self.sim.params.Q_kalman)
            w_psi = np.random.normal(0, 0.1)
            w = np.array([w_x[0], w_y[0], (w_x[1]+w_y[1])/2, w_x[2], w_y[2], (w_x[3]+w_y[3])/2, w_y[4], w_x[4], 0, w_y[5], w_x[5], w_psi, 0])
            solver.y = solver.y + w*(solver.t - solver.t_old)

            timestamps.append(solver.t)
            states.append(solver.y.copy())

            X, u = self.controls.LQR_control_algo(solver.t, solver.y, control_times[-1], self.control_states_cache[-1][-1], self.controls_cache[-1][-1])
            for i in range(5):
                [control_times, kalman_states_x, kalman_states_y, kalman_roll, kalman_cov_x, kalman_cov_y][i].append(X[i])
            self.control_states_cache[-1].append([kalman_states_x[-1], kalman_states_y[-1], kalman_roll[-1], kalman_cov_x[-1], kalman_cov_y[-1]])
            self.controls_cache[-1].append(u)
            self.sim.command = u

            # Real-clock stop. This is not the simulation time in timestamps.
            if time.time() - start_time > 10:
                solver.status = "stopped"
                status = 1
                break
            if (states[-1][2] < 0):
                solver.status = "stopped"
                status = 2
                break
        states = np.array(states).T
        kalman_states_x = np.array(kalman_states_x).T
        kalman_states_y = np.array(kalman_states_y).T
        kalman_roll = np.array(kalman_roll).T

        self.states_cache.append(states)
        self.timestamps_cache.append(timestamps)
        self.control_times_chache.append(control_times)
        self.status_cache.append(status)

    def plot_cache(self, labels=["", "", "", "", "", ""], interval=(0, 0)):
        if interval == (0, 0):
            interval = (0, self.sim.params.t_final)
        Runner.plot(self.timestamps_cache, self.states_cache, labels, True, interval)

    @staticmethod
    def plot(timestamps, states, labels, show, interval):
        fig, ax = plt.subplots(nrows=7, ncols=1, figsize=(8, 20))

        for i in range(len(timestamps)):
            if i != len(timestamps)-1:
                labels = ['', '', '', '', '', '', '', '', '', '', '', '', '']
            else:
                labels = ['x(t)', 'y(t)', 'z(t)', r'$\dot{x}(t)$', r'$\dot{y}(t)$', r'$\dot{z}(t)$', r'$\phi(t)$', r'$\theta(t)$', r'$\psi(t)$', r'$\dot{\phi}(t)$', r'$\dot{\theta}(t)$', r'$\dot{\psi}(t)$', 'm(t)']

            index_i = int(interval[0]/timestamps[0][-1] * len(timestamps[0]))
            index_f = int(interval[1]/timestamps[0][-1] * len(timestamps[0]))

            ax[0].set_ylabel("Height (m)")
            ax[0].plot(timestamps[i][index_i:index_f], states[i][2][index_i:index_f], label=labels[2])

            ax[1].set_ylabel("Velocity (m/s)")
            ax[1].plot(timestamps[i], states[i][5], label=labels[5])

            ax[2].set_ylabel("Angles (rad)")
            ax[2].plot(timestamps[i], states[i][6], label=labels[6])
            ax[2].plot(timestamps[i], states[i][7], label=labels[7])
            ax[2].plot(timestamps[i], states[i][8], label=labels[8])

            ax[3].set_ylabel("Angular velocity (rad/s)")
            ax[3].plot(timestamps[i], states[i][9], label=labels[9])
            ax[3].plot(timestamps[i], states[i][10], label=labels[10])
            ax[3].plot(timestamps[i], states[i][11], label=labels[11])

            ax[4].set_ylabel("Drift (m)")
            ax[4].plot(timestamps[i], states[i][0], label=labels[0])
            ax[4].plot(timestamps[i], states[i][1], label=labels[1])

            ax[5].set_ylabel("Drift velocity (m/s)")
            ax[5].plot(timestamps[i], states[i][3], label=labels[3])
            ax[5].plot(timestamps[i], states[i][4], label=labels[4])

            ax[6].set_ylabel("Mass (kg)")
            ax[6].plot(timestamps[i], states[i][12], label=labels[12])

        for i in range(7):
            ax[i].set_xlabel("Time (s)")
            ax[i].legend()
        if show:
            plt.show()

    def get_cached_flights(self):
        return [self.timestamps_cache, self.states_cache]

    def clear_cache(self):
        self.timestamps_cache = []
        self.states_cache = []
        self.control_times_chache = []
        self.control_states_cache = []
        self.controls_cache = []
        self.status_cache = []
