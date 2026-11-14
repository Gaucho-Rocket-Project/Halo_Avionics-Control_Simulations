"""Closed-loop runner. One accepted integrator step, then the controller.

RK45 takes its own internal substeps inside `solver.step()`. This loop does
not call the controller on those substeps. `max_step` keeps each accepted
step from passing the next control time. The command is stored on the plant
and is used on the next step.
"""
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import RK45


class Runner:
    def __init__(self, controls, sim):
        self.sim = sim
        self.controls = controls
        self.clear_cache()

    def solve(self, State0=None):
        if State0 is None:
            State0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, self.sim.params["m0"]]
       
        initial_command = [1 - (self.sim.params["m0"] * self.sim.params["g"] / self.sim.params["T"]), 0, 0, 0]
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
        # RK45 is adaptive. max_step limits its steps, and the distance to the
        # next control tick makes it land on every scheduled update time.
        max_physics_step = self.controls.dt / 10
        control_tick = 1
        next_control_time = min(control_tick * self.controls.dt, self.sim.params["t_final"])
        time_tolerance = 1e-10 * max(1.0, self.sim.params["t_final"])
        start_time = time.time()
        solver = RK45(self.sim.update, 0, State0, self.sim.params["t_final"], max_step=max_physics_step)
        while solver.status == "running":
            remaining = next_control_time - solver.t
            if remaining <= time_tolerance:
                control_tick += 1
                next_control_time = min(control_tick * self.controls.dt, self.sim.params["t_final"])
                remaining = next_control_time - solver.t
            solver.max_step = min(max_physics_step, remaining)
            solver.step()
            if solver.status == "failed":
                status = 3
                break

            timestamps.append(solver.t)
            states.append(solver.y.copy())

            X, u = self.controls.LQR_control_algo(solver.t, solver.y, control_times[-1], self.control_states_cache[-1][-1], self.controls_cache[-1][-1])
            control_times.append(X[0])
            kalman_states_x.append(X[1])
            kalman_states_y.append(X[2])
            kalman_roll.append(X[3])
            kalman_cov_x.append(X[4])
            kalman_cov_y.append(X[5])
            self.control_states_cache[-1].append([kalman_states_x[-1], kalman_states_y[-1], kalman_roll[-1], kalman_cov_x[-1], kalman_cov_y[-1]])
            self.controls_cache[-1].append(u)
            self.sim.command = u
            # RK45 reuses the derivative from the end of its previous step.
            # Re-evaluate it after the command changes.
            if solver.status == "running":
                solver.f = solver.fun(solver.t, solver.y)
            if solver.t >= next_control_time - time_tolerance:
                control_tick += 1
                next_control_time = min(control_tick * self.controls.dt, self.sim.params["t_final"])

            if time.time() - start_time > 10:
                solver.status = "stopped"
                status = 1
                break
            if states[-1][2] < 0 and solver.t > 1:
                solver.status = "stopped"
                status = 2
                break
        states = np.array(states).T
        kalman_states_x = np.array(kalman_states_x).T
        kalman_states_y = np.array(kalman_states_y).T
        kalman_roll = np.array(kalman_roll).T

        self.states_cache.append(states)
        self.timestamps_cache.append(timestamps)
        self.control_times_cache.append(control_times)
        self.controls_cache[-1] = np.array(self.controls_cache[-1]).T
        self.status_cache.append(status)

    def plot_cache(self, labels=["", "", "", "", "", ""], interval=(0, 0)):
        if interval == (0, 0):
            interval = (0, self.sim.params["t_final"])
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
        self.control_times_cache = []
        self.control_states_cache = []
        self.controls_cache = []
        self.status_cache = []
