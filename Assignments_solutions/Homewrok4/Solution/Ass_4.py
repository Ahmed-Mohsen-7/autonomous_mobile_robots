import sympy
sympy.init_printing(use_latex='mathjax')
from sympy import symbols, Matrix
from IPython.display import display
from numpy import array, sqrt
import numpy as np 
from numpy.random import randn
from math import sqrt, tan, cos, sin, atan2
import matplotlib.pyplot as plt
from math import sqrt
from copy import deepcopy
from matplotlib.patches import Ellipse
import math
from numpy.random import uniform
import scipy.stats

class PFLocalization():
    def __init__(self, prtcls_no,dt, std_vel, std_steer, dim_x=3, dim_z=2, dim_u=2):
        self.dt = dt
        self.std_vel = std_vel
        self.std_steer = std_steer
        self.get_linearized_motion_model()
        self.subs = {self._x: 0, self._y: 0, self._vt:0, self._wt:0, self._dt:dt, self._theta:0}
        self.x = np.zeros((dim_x, 1)) # state
        self.P = np.vstack( np.eye(dim_x) for i in range(prtcls_no))        # uncertainty covariance
        self.R = np.eye(dim_z)        # state uncertainty
        self.Q = np.eye(dim_x)        # process uncertainty
        self.y = np.zeros((dim_z, 1)) # residual
        self.z =  np.array([None]*dim_z)
        self.K = np.zeros(self.x.shape) # kalman gain
        self.y = np.zeros((dim_z, 1))
        self._I = np.eye(dim_x)
        self.ctrl_state = 0        #control state to change ctrl commands
        self.dmin = 0.5   #goal tolerance

    def get_linearized_motion_model(self):
        x, y, theta, vt, wt, dt = sympy.symbols('x, y, theta, v_t, omega_t, delta_t')
        f = sympy.Matrix([[x+-(vt/wt)*sympy.sin(theta)+ (vt/wt)*sympy.sin(theta+ wt*dt)]
                  ,[y+ (vt/wt)*sympy.cos(theta) - (vt/wt)*sympy.cos(theta+ wt*dt)]
                  , [theta+wt*dt]])
        self._x, self._y, self._theta, self._vt, self._wt, self._dt = x, y, theta, vt, wt, dt
        self.state = sympy.Matrix([x, y, theta])
        self.control = sympy.Matrix([vt, wt])
        self.F_j = f.jacobian(self.state)
        self.V_j = f.jacobian(self.control)
        return
    
    def x_forward(self, x, u, dt):
        r = u[0]/u[1]
        theta = x[2]
        rotation = x[2] + u[1]*dt 
        x_plus = np.array([x[0] + -r*sin(theta) + r*sin(rotation),
                       x[1] + r*cos(theta) - r*cos(rotation),
                       x[2] + u[1]*dt])
        return  x_plus 
        
    def get_linearized_measurement_model(self, x, landmark_pos):
        px = landmark_pos[0]
        py = landmark_pos[1]
        hyp = (px - x[0])**2 + (py - x[1])**2
        dist = sqrt(hyp)
        Hx = array([[dist],[atan2(py - x[1], px - x[0]) - x[2]]])
        H = array([[-(px - x[0]) / dist, -(py - x[1]) / dist, 0]
                , [ (py - x[1]) / hyp,  -(px - x[0]) / hyp, -1]])
        return Hx, H
    
    def create_gaussian_particles(self, mean, std, N):
        particles = np.empty((N, 3))
        particles[:, 0] = mean[0] + (randn(N) * std[0])
        particles[:, 1] = mean[1] + (randn(N) * std[1])
        particles[:, 2] = mean[2] + (randn(N) * std[2])
        particles[:, 2] %= 2 * np.pi
        return particles

    def create_uniform_particles(self, x_range, y_range, hdg_range, N):
            particles = np.empty((N, 3))
            particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
            particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
            particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=N)
            particles[:, 2] %= 2 * np.pi
            return particles
        

    def residual(self, a, b):
        """ compute residual (a-b) between measurements containing 
        [range, bearing]. Bearing is normalized to [-pi, pi)"""
        y = a - b
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # to [-pi, pi)
            y[1] -= 2 * np.pi
        return y
    
    def ekf_predict(self, prtcl,idx,u):
        #self.x = self.x_forward(self.x, u, self.dt)
        self.subs[self._x] = prtcl[0]
        self.subs[self._y] = prtcl[1]
        self.subs[self._theta] = prtcl[2]
        self.subs[self._vt] = u[0]
        self.subs[self._wt] = u[1]

        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance in the control space
        M = array([[self.std_vel**2, 0],  [0, self.std_steer**2]])
        prtcl_P_prev =self.P[idx*3:idx*3+3,:]
        #print("F: ",F.shape," | prtcl_P_prev: ",prtcl_P_prev.shape," | V: ",V.shape," | M: ",M.shape)
        prtcl_P = F @ prtcl_P_prev @ F.T + V @ M @ V.T + self.Q
        return prtcl_P
        
    def predict(self, particles, u, std, dt=1.):
        r = u[0]/u[1]
        theta = particles[:, 2]
        rotation = particles[:, 2] + u[1]*dt 
        N = len(particles)
        # TODO update the particle next state based on the motion model defined in self. x_forward()  
        particles[:, 0] = particles[:, 0] + -r*np.sin(theta) + r*np.sin(rotation)
        particles[:, 1] = particles[:, 1] + r*np.cos(theta) - r*np.cos(rotation)
        particles[:, 2] = particles[:, 2] + u[1]*dt 
        particles[:, 2] %= 2 * np.pi

        #apply ekf prediction on each particle
        for idx,prtcl in enumerate(particles):
            #print(particles[idx,:].shape)
            prtcl_P = self.ekf_predict(prtcl,idx,u)
            self.P[idx*3:idx*3+3,:] = prtcl_P
        #print(self.P.shape)
        return particles 
         
    def ekf_update(self,index,particle, z, landmarks):
        Hx, H = self.get_linearized_measurement_model(particle, landmarks)
        PHT = np.dot(self.P[index*3:index*3+3,:], H.T)
        self.K = PHT.dot(np.linalg.inv(np.dot(H, PHT) + self.R))
        self.y = self.residual(z, Hx)
        #print("particle.shape ",particle.shape)
        #print(np.dot(self.K, self.y).shape)
        particle= particle.reshape(3,1) + np.dot(self.K, self.y)
        I_KH = self._I - np.dot(self.K, H)
        self.P[index*3:index*3+3,:] = np.dot(I_KH, self.P[index*3:index*3+3,:] ).dot(I_KH.T) + np.dot(self.K, self.R).dot(self.K.T)
        self.z = deepcopy(z)
        return particle

    def update(self, particles, weights, x, R, landmarks):
        weights.fill(1.)
        # distance from robot to each landmark
        NL = len(landmarks)
        z = (np.linalg.norm(landmarks - x, axis=1) + (randn(NL) * R))
        #print(z)
        #ekf update
        for i, landmark in enumerate(landmarks):
            for index,prtcl in enumerate(particles):
                print(prtcl.shape)
                particles[index,:] = self.ekf_update(index,prtcl,z[i], landmark).reshape(1,3)

        #weights update
        for i, landmark in enumerate(landmarks):
            # TODO calculate measurement residual, i.e., |particles - landmark|
            distance = np.linalg.norm(particles[:, 0:2]-landmark, axis=1)
            # TODO calculate the weighting parameters with respect to each sensor measurement
            #, i.e., scipy.stats.norm(distance, R).pdf(z[i])
            weights *= scipy.stats.norm(distance, R).pdf(z[i])

        weights += 1.e-300      # avoid round-off to zero
        # TODO normalize the weights 
        weights /= sum(weights) 
        return weights
        
    def z_landmark(self, lmark, sim_pos, std_rng, std_brg):
        x, y = sim_pos[0, 0], sim_pos[1, 0]
        d = np.sqrt((lmark[0] - x)**2 + (lmark[1] - y)**2)  
        a = atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
        z = np.array([[d + randn()*std_rng],
                    [a + randn()*std_brg]])
        return z

    def control_strategy(self,x,x_goal):
        r =  np.sqrt(x_goal[0]**2 + x_goal[1]**2) / 4
        done = False
        #move along the first circle
        if (self.ctrl_state == 0 ):
                self.D = np.sqrt((x[0]-x_goal[0]/2)**2 
                                        + (x[1]-x_goal[1]/2)**2)

                if(self.D < self.dmin):
                    print("Reached the intermediate point")
                    self.ctrl_state  = 1
                
                v = 1
                w = -1 / r
                #print("Distance to the goal 1: ", self.D)
        #move along the second circle
        else:      
            self.D = np.sqrt((x[0]-x_goal[0])**2 
                                        + (x[1]-x_goal[1])**2)    
            if(self.D < self.dmin):
                print("Reached the goal")
                done = True
            v = 1
            w = 1 / r
        return v,w,done

    def covariance_ellipse(self, P, deviations=1):
        U, s, _ = np.linalg.svd(P)
        orientation = math.atan2(U[1, 0], U[0, 0])
        width = deviations * math.sqrt(s[0])
        height = deviations * math.sqrt(s[1])
        if height > width:
            raise ValueError('width must be greater than height')
        return (orientation, width, height)
    
    
    def plot_covariance_ellipse(self, mean, cov, std=None, facecolor='b', edgecolor='g', alpha=0.7,ls='solid'):
        ellipse = self.covariance_ellipse(cov)
        ax = plt.gca()
        angle = np.degrees(ellipse[0])
        width = ellipse[1] * 2.
        height = ellipse[2] * 2.
        e = Ellipse(xy=mean, width=std*width, height=std*height, angle=angle, facecolor=facecolor, edgecolor=edgecolor, alpha=alpha, lw=2, ls=ls)
        ax.add_patch(e)
        x, y = mean
        plt.scatter(x, y, marker='+', color=edgecolor)
        a = ellipse[0]
        h, w = height/4, width/4
        plt.plot([x, x+ h*cos(a+np.pi/2)], [y, y + h*sin(a+np.pi/2)])
        plt.plot([x, x+ w*cos(a)], [y, y + w*sin(a)])
        
    def importance_sampling(self, particles, weights, indexes):
        # TODO retrieve selected particle
        particles[:] = particles[indexes]
        # TODO retrieve selected weights
        weights[:] = weights[indexes]
        weights.fill (1.0 / len(weights))
        return  weights
    
    def estimate(self, particles, weights):
        pos = particles[:, 0:2]
        # TODO estimate mean value of the particles 
        mean = np.average(pos, weights=weights, axis=0)
        var  = np.average((pos - mean)**2, weights=weights, axis=0)
        return mean, var
    
    def neff(self, weights):
        return 1. / np.sum(np.square(weights)) 

    def resample_particles(self, weights):
        N = len(weights)
        positions = (np.random.random() + np.arange(N)) / N

        indexes = np.zeros(N, 'i')
        # TODO calculate the cumulative sum of weight distribution 
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes
    
    def run_localization(self, N, landmarks, iteration_num=18, sensor_std_err=.1, initial_x=None):
        plt.figure()
        # create particles and weights
        if initial_x is not None:
            particles = self.create_gaussian_particles(mean=initial_x, std=(5, 5, np.pi/4), N=N)
        else:
            particles = self.create_uniform_particles((-5,5), (-5,5), (-np.pi, -np.pi/2), N)
        weights = np.zeros(N)
          
        xs = []
        self.x = array([[0, 0, -3*np.pi/4]]).T # x, y, steer angle
        sim_pos = self.x.copy()
        u = array([0, .0]) 
        plt.scatter(particles[:, 0], particles[:, 1], alpha=0.1, color='b')
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='s', s=699)
        track = []
        for i in range(iteration_num):
            v,w,done = self.control_strategy(sim_pos, x_goal)  #get control signals based on simulated robot
            u[0],u[1] = v,w 
            sim_pos = self.x_forward(sim_pos, u, self.dt) # simulate robot
            track.append(sim_pos)
            #reach the goal
            if done:
                break
            particles = self.predict(particles, u=u, std=(.02, .05), dt=self.dt)
            # incorporate measurements
            weights = self.update(particles, weights,  sim_pos.flatten()[0:2], R=sensor_std_err, landmarks=landmarks)
            if self.neff(weights) < N/2:
                indexes = self.resample_particles(weights)
                weights = self.importance_sampling(particles, weights, indexes)

            mu, var = self.estimate(particles, weights)
            xs.append(mu)
            # p1 = plt.scatter(sim_pos[0], sim_pos[1], marker='+', color='k', s=180, lw=3)
            # p2 = plt.scatter(mu[0], mu[1], marker='s', color='r')
            # p3 = plt.scatter(particles[:, 0], particles[:, 1], alpha=0.1)
        track = np.array(track)
        xs = np.array(xs)
        p1 = plt.scatter(track[:,0], track[:,1] , color='k',label="Real")
        p2 = plt.scatter(xs[:,0], xs[:,1], color='r',label="PF")
        plt.plot(0,0,'oy',label="Start point")
        plt.plot(D /2,S/2,'ob',label="Intermediate point")
        plt.plot(D ,S,'og',label="Goal point")
        #plt.axis('equal')
        plt.legend()
        print("--- %s seconds ---" % (time.time() - start_time)) 
        plt.show()

import time
start_time = time.time()      
dt = 0.1
#Assumed goal
D = -50
S = 50
x_goal = np.array([[D],[S]])
landmarks = array([[50, 100], [40, 90], [150, 150], [-150, 200]])
prtcls_no = 100
pfl = PFLocalization(prtcls_no,dt, std_vel=1, std_steer=np.radians(1))
#pfl.run_localization(100, landmarks, initial_x=(0, 0, -3*np.pi/4), iteration_num=2000)
pfl.run_localization(prtcls_no, landmarks, iteration_num=2000)

