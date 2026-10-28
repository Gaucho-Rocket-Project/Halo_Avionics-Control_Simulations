#Import libraries and define Symbols
import time
from typing import Any
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import BDF
from control import lqr
import scipy as sp
import numpy.typing as npt

class RocketSim:

    # lambda turning a state into an input matrix
    control_algo: None | Any = None

    timestamps_cache = []
    control_times_chache = []
    states_cache = []
    control_states_cache = []
    controls_cache = []
    status_cache = []
    
    #Class parameters
    T : float                   #Max thrust
    P_max : float               #Max throttle percentage
    m0 : float                  #Initial mass
    m_dot : float               #Mass flow at max thrust
    g = 9.8                     #Gravity
    dNoz : float                #Distance from CoM to Nozzle
    t_final : float             #Final simulation time
    control_frequency : float   #Hertz
    dt : float                  #Control timestep
    l : float                   #Effective length
    r : float                   #Effective radius
    Tau0 : float                #RCS torque magnitude
    State0: list                #Initial conditions
    Q_kalman : npt.NDArray
    


    #initialize with parameters
    def __init__(self,
                T = 3000,                  
                P_max = 0.4,               
                m0 = 285,                  
                m_dot = 1.75,                                   
                dNoz = 0.7,                
                t_final = 65,                               
                l = 2.5,                   
                r = 0.2,                   
                Tau0 = 3.23,
                State0 = [0,0,0,
                          0,0,0,
                          0,0,0,
                          0,0,0]):
        self.T = T                  
        self.P_max = P_max
        self.m0 = m0
        self.m_dot = m_dot
        self.dNoz = dNoz
        self.t_final = t_final
        self.l = l
        self.r = r
        self.Tau0 = Tau0
        self.State0 = State0.append(m0)
        
        self.position_variance = 1e-10
        self.velocity_variance = 1e-5
        self.Q_kalman = np.diag([self.position_variance, self.position_variance, self.velocity_variance, self.velocity_variance, 1e-10, 1e-7])
        
        self.clear_cache()

    def set_params(self, T, P_max, m0, m_dot, dNoz, t_final, l, r, Tau0):
        self.T = T                  
        self.P_max = P_max
        self.m0 = m0
        self.m_dot = m_dot
        self.dNoz = dNoz
        self.t_final = t_final
        self.l = l
        self.r = r
        self.Tau0 = Tau0
        #self.State0[12] = m0
        

    def with_control_algo(self, algo) -> Any:
        self.control_algo = algo
        return self


    # Define the system of ODEs
    def update(self, t, state):
        x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, m = (state)
        P, gx, gy, f = self.controls_cache[-1][-1]
        
        dm = (-self.m_dot/self.T) * self.T * (1-P)

        Ixx = (1/12) * m * (self.l ** 2)
        Iyy = (1/12) * m * (self.l ** 2)
        Izz = (1/2) * m * (self.r ** 2)
        dIxx = (1/12) * dm * (self.l ** 2)
        dIyy = (1/12) * dm * (self.l ** 2)
        dIzz = (1/2) * dm * (self.r ** 2)

        ddx = -self.T*(P - 1)*(((np.sin(phi)*np.sin(psi) + np.sin(theta)*np.cos(phi)*np.cos(psi))*np.cos(gy) + np.sin(gy)*np.cos(psi)*np.cos(theta))*np.cos(gx) - (np.sin(phi)*np.sin(theta)*np.cos(psi) - np.sin(psi)*np.cos(phi))*np.sin(gx))/m
        ddy = self.T*(P - 1)*(((np.sin(phi)*np.cos(psi) - np.sin(psi)*np.sin(theta)*np.cos(phi))*np.cos(gy) - np.sin(gy)*np.sin(psi)*np.cos(theta))*np.cos(gx) + (np.sin(phi)*np.sin(psi)*np.sin(theta) + np.cos(phi)*np.cos(psi))*np.sin(gx))/m
        ddz = (self.T*(P - 1)*((np.sin(gy)*np.sin(theta) - np.cos(gy)*np.cos(phi)*np.cos(theta))*np.cos(gx) + np.sin(gx)*np.sin(phi)*np.cos(theta)) - self.g*m)/m
        ddphi = (Ixx*Iyy*Izz*dtheta*(dphi*np.tan(theta) + dpsi/np.cos(theta)) - Ixx*Iyy*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)))*np.cos(phi)*np.tan(theta) + Ixx*Izz*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + self.T*self.dNoz*(P - 1)*np.sin(gy)*np.cos(gx) - dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.sin(phi)*np.tan(theta) + Iyy*Izz*(Iyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) - Izz*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + self.T*self.dNoz*(P - 1)*np.sin(gx) - dIxx*(dphi - dpsi*np.sin(theta))))/(Ixx*Iyy*Izz)
        ddtheta = (-Iyy*Izz*dphi*dpsi*np.cos(theta) + Iyy*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)))*np.sin(phi) + Izz*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + self.T*self.dNoz*(P - 1)*np.sin(gy)*np.cos(gx) - dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.cos(phi))/(Iyy*Izz)
        ddpsi = (Iyy*Izz*dtheta*(dphi + dpsi*np.sin(theta)) - Iyy*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + Iyy*(dphi - dpsi*np.sin(theta))*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)) + dIzz*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)))*np.cos(phi) + Izz*(-Ixx*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + Izz*(dphi - dpsi*np.sin(theta))*(dpsi*np.cos(phi)*np.cos(theta) - dtheta*np.sin(phi)) + self.T*self.dNoz*(P - 1)*np.sin(gy)*np.cos(gx) - dIyy*(dpsi*np.sin(phi)*np.cos(theta) + dtheta*np.cos(phi)))*np.sin(phi))/(Iyy*Izz*np.cos(theta)) + (self.Tau0/Izz) * f 

        return [dx, dy, dz, ddx, ddy, ddz, dphi, dtheta, dpsi, ddphi, ddtheta, ddpsi, dm]
 
    #def system(self):
    #    return lambda t, state: self.update(state, self.control_algo(t, state))
    
    def solve(self, State0 = [0,0,0,
                          0,0,0,
                          0,0,0,
                          0,0,0, 285]):
        if self.control_algo is None:
            raise Exception("Control Algorithm Not Configured")
        self.controls_cache.append([[0,0,0,0]]) #control values
        control_times = [0]

        kalman_states_x = [np.array([State0[i] for i in [0,2,3,5,7,10]]) * np.array([1,1,1,1,-1,-1])]
        kalman_states_y = [np.array([State0[i] for i in [1,2,4,5,6,9]]) * np.array([-1,1,-1,1,-1,-1])]
        kalman_roll = [[0,0]]
        kalman_cov_x = [np.zeros((6,6))]
        kalman_cov_y = [np.zeros((6,6))]
        self.control_states_cache.append([[kalman_states_x[-1], kalman_states_y[-1], kalman_roll[-1], kalman_cov_x[-1], kalman_cov_y[-1]]])


        timestamps = []
        states = []
        #Start Solver
        status = 0
        start_time = time.time()
        solver = BDF(self.update, 0, State0, self.t_final)
        while solver.status == "running":
            #Step solver and save data
            solver.step()

            #Add disturbance
            w_x = np.random.multivariate_normal(np.zeros(6), self.Q_kalman)
            w_y = np.random.multivariate_normal(np.zeros(6), self.Q_kalman)
            w_psi = np.random.normal(0,0.1) 
            w = np.array([w_x[0],w_y[0], (w_x[1]+w_y[1])/2, w_x[2], w_y[2], (w_x[3]+w_y[3])/2, w_y[4], w_x[4], 0, w_y[5], w_x[5],w_psi,0])
            solver.y = solver.y + w*(solver.t - solver.t_old)

            timestamps.append(solver.t)
            states.append(solver.y.copy())

            X, u = self.control_algo(solver.t, solver.y, control_times[-1], self.control_states_cache[-1][-1], self.controls_cache[-1][-1])
            for i in range(5):
                [control_times, kalman_states_x, kalman_states_y, kalman_roll, kalman_cov_x, kalman_cov_y][i].append(X[i])
            self.control_states_cache[-1].append([kalman_states_x[-1], kalman_states_y[-1], kalman_roll[-1], kalman_cov_x[-1], kalman_cov_y[-1]])
            self.controls_cache[-1].append(u)
            #[[t, kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y], [P, gx, gy, f]]
            #End conditions
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
        #kalman_states = np.array([kalman_states_x, kalman_states_y, kalman_roll, kalman_cov_x, kalman_cov_y])

        self.states_cache.append(states)
        self.timestamps_cache.append(timestamps)
        #self.control_states_cache.append(kalman_states)
        self.control_times_chache.append(control_times)
        self.status_cache.append(status)
    
    def plot_cache(self, labels = ["", "", "", "", "", ""], interval = (0,0)):
        if interval == (0,0):
            interval = (0, self.t_final)
        RocketSim.plot(self.timestamps_cache, self.states_cache, labels, True, interval)

    @staticmethod
    def plot(timestamps, states, labels, show, interval):
        #Plot the trajectory
        fig, ax = plt.subplots(nrows=7, ncols=1,figsize=(8, 20))
        #labels = ['x(t)', 'y(t)', 'z(t)', r'$\dot{x}(t)$', r'$\dot{y}(t)$', r'$\dot{z}(t)$', r'$\phi(t)$', r'$\theta(t)$', r'$\psi(t)$', r'$\dot{\phi}(t)$', r'$\dot{\theta}(t)$', r'$\dot{\psi}(t)$', 'm(t)']

        for i in range(len(timestamps)):
            if i != len(timestamps)-1:
                labels = ['','','','','','','','','','','','','']
            else:
                labels = ['x(t)', 'y(t)', 'z(t)', r'$\dot{x}(t)$', r'$\dot{y}(t)$', r'$\dot{z}(t)$', r'$\phi(t)$', r'$\theta(t)$', r'$\psi(t)$', r'$\dot{\phi}(t)$', r'$\dot{\theta}(t)$', r'$\dot{\psi}(t)$', 'm(t)']

            index_i = int(interval[0]/timestamps[0][-1] * len(timestamps[0]))
            index_f = int(interval[1]/timestamps[0][-1] * len(timestamps[0]))

            ax[0].set_ylabel("Height (m)")
            ax[0].plot(timestamps[i][index_i:index_f], states[i][2][index_i:index_f], label = labels[2])
            #ax[0].plot(kalman_times, kalman_states_x[1]) #kalman estimates
            #ax[0].plot(timestamps[i], [z_ref(i) for i in timestamps[i]], 'g') #target trajectory


            ax[1].set_ylabel("Velocity (m/s)")
            ax[1].plot(timestamps[i], states[i][5], label = labels[5])
            #ax[1].plot(kalman_times, kalman_states_x[3]) #kalman estimates
            #ax[1].plot(timestamps[i], [dz_ref(i) for i in timestamps[i]],'g') #target trajectory

            ax[2].set_ylabel("Angles (rad)")
            ax[2].plot(timestamps[i], states[i][6], label = labels[6])
            ax[2].plot(timestamps[i], states[i][7], label = labels[7])
            ax[2].plot(timestamps[i], states[i][8], label = labels[8])

            ax[3].set_ylabel("Angular velocity (rad/s)")
            ax[3].plot(timestamps[i], states[i][9], label = labels[9])
            ax[3].plot(timestamps[i], states[i][10], label = labels[10])
            ax[3].plot(timestamps[i], states[i][11], label = labels[11])

            ax[4].set_ylabel("Drift (m)")
            ax[4].plot(timestamps[i], states[i][0], label = labels[0])
            ax[4].plot(timestamps[i], states[i][1], label = labels[1])

            ax[5].set_ylabel("Drift velocity (m/s)")
            ax[5].plot(timestamps[i], states[i][3], label = labels[3])
            ax[5].plot(timestamps[i], states[i][4], label = labels[4])

            ax[6].set_ylabel("Mass (kg)")
            ax[6].plot(timestamps[i], states[i][12], label = labels[12])

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
    
class Controls():
    #Flight Parameters
    H : float   #Hover height
    V : float     #Velocity of ascension/descension
    Ti : float   #Initial hovering time
    Th : float   #Hover duration
    Tf : float   #Landing time
    Q : npt.NDArray
    R : list
    K : list
    control_frequency : float
    dt : float
    Atc : list
    Btc : list
    

    def __init__(self,
                H = 50.0,   
                V = 4.0,    
                Ti = 21.0,  
                Th = 10.0, 
                Tf = 54.0,
                Q = np.diag([0.1,350,0.1,320,20,50]),
                R = np.diag([2000,1536]),
                control_frequency = 15,
                rocket=RocketSim()):
        self.H = H                  
        self.V = V
        self.Ti = Ti
        self.Th = Th
        self.Tf = Tf
        self.R = R
        self.control_frequency = control_frequency
        self.dt = 1/control_frequency
        #x1, x2, x3=Ti, x4, x5, x6, X, Y, Z
        self.Z = 0.83
        self.x1 = self.V / self.Z
        self.x2 = (2/self.V) * (self.H+self.V * self.x1 - 0.5* self.Z * (self.x1**2) ) - self.Ti
        self.x4 = self.Ti +self.Th
        self.x5 = self.x4 + (self.Ti - self.x2)
        self.X = self.V/(2*(self.Ti-self.x2))
        self.x6 = (2/self.V) * (self.H+self.V * self.x5 - self.X* ((self.x5-self.x4) **2) ) - self.Tf
        self.Y = self.V/(2*(self.Tf-self.x6))
        #Process and measurement covariances
        self.Q = Q
        self.R_kalman = np.diag([1, 1, 1, 1, 1, 1]) * 1e-5 
        self.rocket_sim_instance = rocket
        self.Q_d = np.identity(self.rocket_sim_instance.Q_kalman.shape[0]) + self.rocket_sim_instance.Q_kalman * self.dt
        self.R_d = self.R_kalman / self.dt
        self.Iyy = (1/12) * self.rocket_sim_instance.m0 * (self.rocket_sim_instance.l ** 2)
        self.Izz = (1/2) * self.rocket_sim_instance.m0 * (self.rocket_sim_instance.r ** 2)
       


        #Continuous control matrices
        self.Atc = np.zeros([6,6])
        self.Atc[0,2] = 1
        self.Atc[1,3] = 1
        self.Atc[2,4] = -self.rocket_sim_instance.g
        self.Atc[4,5] = 1

        self.Btc = np.zeros([6,2])
        self.Btc[2,0] = -self.rocket_sim_instance.g
        self.Btc[3,1] = -self.rocket_sim_instance.T/self.rocket_sim_instance.m0
        
        self.Btc[5,0] = self.rocket_sim_instance.dNoz*self.rocket_sim_instance.m0*self.rocket_sim_instance.g/self.Iyy

        #Discrete control matrices
        Concatenated = np.concatenate((np.concatenate((self.Atc, self.Btc), axis = 1), np.zeros((2,8))),axis=0)
        Exp_Concat = np.round(sp.linalg.expm(Concatenated * self.dt), decimals=4)
        self.Atd = Exp_Concat[:6,:6]
        self.Btd = Exp_Concat[:6,6:]

        self.K = lqr(self.Atc, self.Btc, self.Q, R)[0]

    def set_params(self, H, V, Ti, Th, Tf, Z, Q, R, control_frequency, rocket):
        self.V = V
        self.Ti = Ti
        self.Th = Th
        self.Tf = Tf
        self.R = R
        self.control_frequency = control_frequency
        self.dt = 1/control_frequency
        #x1, x2, x3=Ti, x4, x5, x6, X, Y, Z
        self.Z = Z
        self.x1 = self.V / self.Z
        self.x2 = (2/self.V) * (self.H+self.V * self.x1 - 0.5* self.Z * (self.x1**2) ) - self.Ti
        self.x4 = self.Ti +self.Th
        self.x5 = self.x4 + (self.Ti - self.x2)
        self.X = self.V/(2*(self.Ti-self.x2))
        self.x6 = (2/self.V) * (self.H+self.V * self.x5 - self.X* ((self.x5-self.x4) **2) ) - self.Tf
        self.Y = self.V/(2*(self.Tf-self.x6))
        #Process and measurement covariances
        self.Q = Q
        self.R_kalman = np.diag([1, 1, 1, 1, 1, 1]) * 1e-5 
        self.rocket_sim_instance = rocket
        self.Q_d = np.identity(self.rocket_sim_instance.Q_kalman.shape[0]) + self.rocket_sim_instance.Q_kalman * self.dt
        self.R_d = self.R_kalman / self.dt
        self.Iyy = (1/12) * self.rocket_sim_instance.m0 * (self.rocket_sim_instance.l ** 2)
        self.Izz = (1/2) * self.rocket_sim_instance.m0 * (self.rocket_sim_instance.r ** 2)

        #Continuous control matrices
        self.Atc = np.zeros([6,6])
        self.Atc[0,2] = 1
        self.Atc[1,3] = 1
        self.Atc[2,4] = -self.rocket_sim_instance.g
        self.Atc[4,5] = 1

        self.Btc = np.zeros([6,2])
        self.Btc[2,0] = -self.rocket_sim_instance.g
        self.Btc[3,1] = -self.rocket_sim_instance.T/self.rocket_sim_instance.m0
        
        self.Btc[5,0] = self.rocket_sim_instance.dNoz*self.rocket_sim_instance.m0*self.rocket_sim_instance.g/self.Iyy

        #Discrete control matrices
        Concatenated = np.concatenate((np.concatenate((self.Atc, self.Btc), axis = 1), np.zeros((2,8))),axis=0)
        Exp_Concat = np.round(sp.linalg.expm(Concatenated * self.dt), decimals=4)
        self.Atd = Exp_Concat[:6,:6]
        self.Btd = Exp_Concat[:6,6:]

        self.K = lqr(self.Atc, self.Btc, self.Q, R)[0]


    
    def z_ref(self, t):
        if t< self.x1:
            return 0.5 * self.Z * (t **2)
        elif (t>self.x1) and (t<self.x2):
            return self.V * (t- self.x1) + 0.5 * self.Z * (self.x1 **2)
        elif (t>self.x2) and (t<self.Ti):
            return -self.X * ((t-self.Ti) **2) + self.H
        elif (t>self.Ti) and (t<self.x4):
            return self.H
        elif (t>self.x4) and (t<self.x5):
            return -self.X * ((t-self.x4) ** 2) + self.H
        elif (t>self.x5) and (t<self.x6):
            return -self.V * (t-self.x5) + self.H -self.X * ((self.x5-self.x4) ** 2)
        elif (t>self.x6) and (t<self.Tf):
            return self.Y * ((t-self.Tf) ** 2)
        else:
            return 0

    def dz_ref(self, t):
        return (self.z_ref(t+0.001) - self.z_ref(t)) / 0.001
    

    #LQR takes in estimated state and outputs gimbal and throttle controls
    def LQR(self, t_hat, State_hat, u):
        x_hat, z_hat, dx_hat, dz_hat, theta_hat, dtheta_hat = State_hat
        gamma0, P0 = u

        #Set control parameters
        gamma, P = -self.K @ [x_hat, z_hat-self.z_ref(t_hat), dx_hat, dz_hat-self.dz_ref(t_hat), theta_hat, dtheta_hat]

        #Control limits
        #Throttle
        bP=1.0 #damping
        #max throttle rate
        dP = P-P0
        P = P0 + np.clip((dP/self.dt),-0.5,0.5)*self.dt / bP #update P with damping
        P = np.clip(P,0,self.rocket_sim_instance.P_max) #min/max throttle

        #Gimbal
        bgamma = 1.0 #gimbal damping
        #max gimbal rate
        dgamma = gamma-gamma0
        gamma = gamma0 + np.clip((dgamma/self.dt),-0.5, 0.5)*self.dt / bgamma #update P with damping
        gamma = np.clip(gamma, -0.22, 0.22) #min/max gimbal

        return [gamma, P]#, np.clip((dgamma/self.dt),-0.5, 0.55), np.clip((dP/self.dt),-0.5,0.5)]



    #Kalman filter takes in observed state and uncertainty and outputs estimated state
    def Kalman_Filter(self, observed_state, kalman_state, kalman_covariance, u):
        apriori_state = self.Atd@kalman_state + self.Btd@u
        apriori_covariance = self.Atd@kalman_covariance@self.Atd.T + self.Q_d
        kalman_gain = (apriori_covariance) @np.linalg.inv(apriori_covariance + self.R_d)
            
        aposteriori_state = apriori_state + kalman_gain@(observed_state - apriori_state)
        aposteriori_covariance = (np.identity(6) - kalman_gain)@apriori_covariance@(np.identity(6) - kalman_gain).T + kalman_gain@self.R_d@kalman_gain.T

        return [aposteriori_state, aposteriori_covariance]

    #Roll control (subject to change)
    def Roll_control(self, psi, dpsi, f_prev):
        ta = ( (self.Izz/self.rocket_sim_instance.Tau0) * dpsi * np.sign(psi) + np.sqrt( (self.Izz/self.rocket_sim_instance.Tau0) * np.abs(psi) + 0.5 * (((self.Izz/self.rocket_sim_instance.Tau0) * dpsi) ** 2) ) )
        tb = ( (self.Izz/self.rocket_sim_instance.Tau0) * dpsi * np.sign(psi) + 2 * np.sqrt( (self.Izz/self.rocket_sim_instance.Tau0) * np.abs(psi) + 0.5 * (((self.Izz/self.rocket_sim_instance.Tau0) * dpsi) ** 2) ) )
        k=2
        e_on = 0.02
        e_off = 0.02 * 0.3
        if f_prev == 0:
            if abs(psi+k*dpsi)> e_on:
                if np.ceil(ta/(self.dt*1))*(self.dt*1)>0:
                    f = -np.sign(psi)
                elif np.ceil(tb/(self.dt*1))*(self.dt*1)>0:
                    f = np.sign(psi)
                else:
                    f = 0
            else:
                f= 0
        else:
            if abs(psi+k*dpsi)< e_off:
                f=0
            else:
                if np.ceil(ta/(self.dt*1))*(self.dt*1)>0:
                    f = -np.sign(psi)
                elif np.ceil(tb/(self.dt*1))*(self.dt*1)>0:
                    f = np.sign(psi)
                else:
                    f = 0
        return f
    
    def LQR_control_algo(self, t, state, control_t_prev, control_state_prev, u_prev):
        if (t - control_t_prev > self.dt):
            #Measurement
            observed_state_x = [state[i] for i in [0,2,3,5,7,10]] * np.array([1,1,1,1,-1,-1])
            observed_state_y = [state[i] for i in [1,2,4,5,6,9]] * np.array([-1,1,-1,1,-1,-1])
            observed_roll = [state[i] for i in [8,11]]

            v = np.random.multivariate_normal(np.zeros(6), self.R_kalman)
            observed_state_x += self.R_d@v
            observed_state_y += self.R_d@v

            kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y = control_state_prev
            P, gx, gy, f = u_prev

            #Kalman filter
            aposteriori_state_x, aposteriori_covariance_x = self.Kalman_Filter(observed_state_x, kalman_state_x, kalman_cov_x, [gy, P])
            aposteriori_state_x = aposteriori_state_x * np.array([1,1,1,1,-1,-1])
            kalman_state_x = aposteriori_state_x
            kalman_cov_x = aposteriori_covariance_x

            aposteriori_state_y, aposteriori_covariance_y = self.Kalman_Filter(observed_state_y, kalman_state_y, kalman_cov_y, [gx, P])
            aposteriori_state_y = aposteriori_state_y * np.array([-1,1,-1,1,-1,-1])
            kalman_state_y = aposteriori_state_y
            kalman_cov_y = aposteriori_covariance_y


            #LQR
            gy, P1 = self.LQR(t, kalman_state_x * np.array([1,1,1,1,-1,-1]), [gy, P])
            gx, P2 = self.LQR(t, kalman_state_y * np.array([-1,1,-1,1,-1,-1]), [gx, P])
            P = (P1+P2)/2

            #Roll control
            kalman_roll = np.array(observed_roll)
            psi = observed_roll[0]
            dpsi = observed_roll[1]
            f = self.Roll_control(psi, dpsi, f)

            return [[t, kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y], [P, gx, gy, f]]
        else:
            kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y = control_state_prev
            return [[control_t_prev, kalman_state_x, kalman_state_y, kalman_roll, kalman_cov_x, kalman_cov_y], u_prev]
    
    #x, y, z, dx, dy, dz, phi, theta, psi, dphi, dtheta, dpsi, m = (state)
    #P, gx, gy, f = self.controls_cache[-1][-1]
    #t, State_x_hat, State_y_hat, Cov_x, Cov_y = X
    #X, u = self.control_algo(solver.t, solver.y, control_times[-1], self.control_states_cache[-1], self.controls_cache[-1])
