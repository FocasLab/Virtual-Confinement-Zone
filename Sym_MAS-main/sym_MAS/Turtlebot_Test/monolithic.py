import numpy as np
import numba as nb
import time

from scipy.io import loadmat,savemat
from scipy.sparse import csr_matrix
from scipy import sparse
from math import pi, sqrt
from sys import exit

# For Dynamics
tau = 2.1

bound_x = np.array([0, 5])
bound_y = np.array([0, 5])
bound_theta = np.array([-pi, pi])
bound = np.array([bound_x, bound_y, bound_theta])

bound_v = np.array([-0.22, 0.22])
bound_w = np.array([-0.11, 0.11])
bound_u = np.array([bound_v, bound_w])

print("bound_x:", bound_x.shape)
print("bound_y:", bound_y.shape)
print("bound_theta:", bound_theta.shape)
print("bound:", bound.shape)
print(bound[:,0].shape)
print("bound_v:", bound_v.shape)
print("bound_w:", bound_w.shape)
print("bound_u:", bound_u.shape)

# Space discretization
# numbers of intervals
n_x = 5
n_y = 5
n_theta = 5

# size of the interval
d_x = (bound_x[1] - bound_x[0]) / (n_x)
d_y = (bound_y[1] - bound_y[0]) / (n_y)
d_theta = (bound_theta[1] - bound_theta[0]) / (n_theta)
d_states = np.array([d_x, d_y, d_theta])

# Input discretization
n_v = 5
n_w = 5
d_v = (bound_v[1] - bound_v[0]) / (n_v)
d_w = (bound_w[1] - bound_w[0]) / (n_w)

# Distance between robots
d = 1

u_values1 = np.arange(bound_u[0][0],bound_u[0][1] + (bound_u[0][1] - bound_u[0][0]) / (n_v),(bound_u[0][1] - bound_u[0][0]) / (n_v))
u_values1 = u_values1[0:6] 
u_values2 = np.arange(bound_u[1][0],bound_u[1][1] + (bound_u[1][1] - bound_u[1][0]) / (n_w),(bound_u[1][1] - bound_u[1][0]) / (n_v))
u_values2 = u_values2[0:6] 

print("u_values1", u_values1)
print("u_values2", u_values2)
print("d_x: ", d_x, " d_y: ", d_y, " d_theta :", d_theta)
print("d_v: ", d_v, " d_w: ", d_w)


ELL = 0.2

def diff_Successor(curr_state,input,time_step):
    # Set the track of the vehicle [m]
    # input = uni2diff(input)
    new_state = rk_four(diffdrive_f,curr_state,input,time_step)

    return new_state  

def rk_four(f, x, u, T):
    """Fourth-order Runge-Kutta numerical integration."""
    k_1 = f(x, u)
    k_2 = f(x + T * k_1 / 2.0, u)
    k_3 = f(x + T * k_2 / 2.0, u)
    k_4 = f(x + T * k_3, u)
    x_new = x + T / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4)
    return x_new

def diffdrive_f(x, u):
    """Differential drive kinematic vehicle model."""
    f = np.zeros(3)
    f[0] = (u[0]) * np.cos(x[2])
    f[1] = (u[0]) * np.sin(x[2])
    f[2] = (u[1])
    return f

# def uni2diff(u):
#     """Convert speed and angular rate to wheel speeds."""
#     v = u[0]
#     omega = u[1]
#     v_L = v - ELL / 2 * omega
#     v_R = v + ELL / 2 * omega
#     return np.array([v_L, v_R]

Delta = csr_matrix(((n_x*n_y*n_theta)*(n_x*n_y*n_theta)*(n_v)*(n_w)*(n_v)*(n_w), (n_x*n_y*n_theta)*(n_x*n_y*n_theta)), dtype=bool)
# Delta = Delta2.tolil()
# @nb.jit(nopython=True, nogil=True)
def compute_succ(Delta):
    for i1 in range(n_x):
        print("Progress:- ", (i1 / n_x) * 100)
        for i2 in range(n_y):
            for i3 in range(n_theta):
                for i4 in range(n_x):
                    for i5 in range(n_y):
                        for i6 in range(n_theta):
                            if sqrt((i1-i4)**2+(i2-i5)**2)-d>=0:
                                bound_a1 = np.array([bound[:,0]]).T
                                bound_a2 = np.array([bound[:,0]]).T
                                a1 = np.multiply(np.array([i1, i2, i3+bound[2][0]/d_theta]),np.array([d_states])).T
                                a2 = np.multiply(np.array([i1+1, i2+1, i3+1+bound[2][0]/d_theta]),np.array([d_states])).T
                                a1 = bound_a1 + a1
                                a2 = bound_a2 + a2
                                bound_a3 = np.array([bound[:,0]]).T
                                bound_a4 = np.array([bound[:,0]]).T
                                a3 = np.multiply(np.array([i4, i5, i6+bound[2][0]/d_theta]),np.array([d_states])).T
                                a4 = np.multiply(np.array([i4+1, i5+1, i6+1+bound[2][0]/d_theta]),np.array([d_states])).T
                                a3 = bound_a3 + a3
                                a4 = bound_a4 + a4
                                x = np.concatenate((a1,a2,a3,a4),axis=1)                        
                                for h1 in range(n_v):
                                    for h2 in range(n_w):
                                        for h3 in range(n_v):
                                            for h4 in range(n_w):
                                                u_valuesr1 = np.array([u_values1[h1],u_values2[h2]])
                                                u_valuesr2 = np.array([u_values1[h3],u_values2[h4]])
                                                # Successor
                                                xp1 = diff_Successor(x[:,0],u_valuesr1,tau)
                                                xp2 = diff_Successor(x[:,1],u_valuesr1,tau)
                                                xp3 = diff_Successor(x[:,2],u_valuesr2,tau)
                                                xp4 = diff_Successor(x[:,3],u_valuesr2,tau)

                                                if ((all(np.greater_equal(xp1,bound[:,0]))) and (all(np.less_equal(xp2,bound[:,1]))) and (all(np.greater_equal(xp3,bound[:,0]))) and (all(np.less_equal(xp4,bound[:,1]))) and ((sqrt((xp1[0]-xp3[0])**2+(xp1[1]-xp3[1])**2)-d)>=0) and ((sqrt((xp2[0]-xp4[0])**2+(xp2[1]-xp4[1])**2)-d)>=0)):
                                                    # associate successors of extrema to extremal intervals
                                                    minsuccr1 = np.floor((xp1-bound[:,0])/d_states)
                                                    maxsuccr1 = np.ceil((xp2-bound[:,0])/d_states)
                                                    minsuccr2 = np.floor((xp3-bound[:,0])/d_states)
                                                    maxsuccr2 = np.ceil((xp4-bound[:,0])/d_states)
                                                    for ip1 in range(int(minsuccr1[0]),int(maxsuccr1[0])):
                                                        for ip2 in range(int(minsuccr1[1]),int(maxsuccr1[1])):
                                                            for ip3 in range(int(minsuccr1[2]),int(maxsuccr1[2])):
                                                                for ip4 in range(int(minsuccr2[0]),int(maxsuccr2[0])):
                                                                    for ip5 in range(int(minsuccr2[1]),int(maxsuccr2[1])):
                                                                        for ip6 in range(int(minsuccr2[2]),int(maxsuccr2[2])):
                                                                            row = ((i1)*n_y*n_theta*n_x*n_y*n_theta*n_v*n_w*n_v*n_w + (i2)*n_theta*n_x*n_y*n_theta*n_v*n_w*n_v*n_w+ (i3)*n_x*n_y*n_theta*n_v*n_w*n_v*n_w+ (i4)*n_y*n_theta*n_v*n_w*n_v*n_w+ i5*n_theta*n_v*n_w*n_v*n_w+ i6*n_v*n_w*n_v*n_w + (h1)*n_w*n_v*n_w+ (h2)*n_v*n_w + h3*n_w+ h4)
                                                                            # print(Delta.shape)
                                                                            # print("Row",row)
                                                                            col = ((ip1)*n_y*n_theta*n_x*n_y*n_theta + (ip2)*n_theta*n_x*n_y*n_theta + ip3*n_x*n_y*n_theta+ ip4*n_y*n_theta+ip5*n_theta+ip6)
                                                                            # print("Column",col)
                                                                            Delta[row,col]=1
    print(np.count_nonzero(Delta))
# Controller Synthesis
def controller_synthesis(composed_delta):
    Cont = np.zeros((n_x*n_y*n_theta*n_x*n_y*n_theta,n_v*n_w*n_v*n_w),dtype=bool) #create the matrix structure (defined only with 0,1) 
    for i in range(n_x*n_y*n_theta*n_x*n_y*n_theta): #checking admissible modes for any state
        print("Progress:- ", (i / n_x*n_y*n_theta*n_x*n_y*n_theta) * 100)
        for h in range(n_v*n_w*n_v*n_w): #for any control mode
            succ = np.where(composed_delta[i*n_v*n_w*n_v*n_w+h][:] == 1)
            if succ[0].size > 0:
                Cont[i][h] = 1
    return Cont

# For Safety Specifications
def safety_controller(Controller,composed_delta):
    Contp = np.zeros((n_x*n_y*n_theta,n_v*n_w),dtype=bool) #create the matrix structure (defined only with 0,1) 
    iter=0
    while (not (np.array_equal(Contp, Controller))):
        iter += 1
        Contp = np.copy(Controller)
        for i in range(n_x*n_y*n_theta):
            print("Progress:- ", (i / (n_x*n_y*n_theta)) * 100)
            mode = np.where(Contp[i][:] == 1)
            if mode[0].size > 0:
                for k in range(len(mode[0])):
                    val = mode[0][k]
                    succ = np.where(composed_delta[i*n_v*n_w + val][:] == 1)
                    if succ[0].size > 0:
                        for ip in range(len(succ[0])):
                            val1 = succ[0][ip]
                            if ~np.any(Contp[val1] == 1):
                                Controller[i][val] = 0
                                break
    return Controller

# For Reachability
def reach_controller(composed_delta,Cont_syn):
    Cont_syn_orig = np.copy(Cont_syn)
    Cont = np.copy(Cont_syn)

    rxr1 = np.array([4, 5])
    ryr1 = np.array([4, 5])
    rxr2 = np.array([0, 1])
    ryr2 = np.array([0, 1])
    Contp = np.zeros((n_x*n_y*n_theta*n_x*n_y*n_theta,n_v*n_w*n_v*n_w),dtype=bool) #create the matrix structure (defined only with 0,1)
    reach_states = np.array([])

    for i in range(2):
        for j in range(2):
            for t1 in range(5):
                for k in range(2):
                    for l in range(2):
                        for t2 in range(5):
                            reach_states=np.append(reach_states,rxr1[i]*n_y*n_theta*n_x*n_y*n_theta+ryr1[j]*n_theta*n_x*n_y*n_theta+t1*n_x*n_y*n_theta+rxr2[k]*n_y*n_theta+ryr2[l]*n_theta+t2)
    iter=0
    while (not (np.array_equal(Contp, Cont))):
        iter += 1
        Contp = np.copy(Cont)
        for i in range(n_x*n_y*n_theta*n_x*n_y*n_theta):
            print("Progress:- ", (i / (n_x*n_y*n_theta)) * 100)
            mode = np.where(Cont_syn_orig[i][:] == 1)
            if mode[0].size > 0:
                for k in range(len(mode[0])):
                    val = mode[0][k]
                    succ = np.where(composed_delta[i*n_v*n_w*n_v*n_w + val][:] == 1)
                    if succ[0].size > 0:    
                        for ip in range(len(succ[0])):
                            val1 = succ[0][ip]

                            if ~(np.isin(i,reach_states)):
                                if ~(np.isin(val1,reach_states)):
                                    Cont[i][val] = 0
                                    break

                                else:
                                    Cont[i][val] = 1
                                    reach_states = np.append(reach_states,i)
    return Cont

# Compute Automata ( reachable sets )
print("Step 1: Computation of the abstraction's transition relation")
start_time = time.time()
control_delta = compute_succ(Delta)
# savemat("path",{'control_delta':control_delta})
print("--- Abstraction Transition total Completion Time %s seconds -- \n" % (time.time() - start_time))

print("Step 2: Computation of Controller")
start_time = time.time()
controller_syn = controller_synthesis(control_delta)
# savemat("path",{'controller_syn':controller_syn})

# print("Step 2: Controller Synthesis for Safety Specification \n")
# start_time = time.time()
# safe_controller = safety_controller(controller_syn,control_delta)
# savemat("path",{'safe_controller':safe_controller})
# print("--- Total Safety Controller Compute Time %s seconds --- \n" % (time.time() - start_time))

print("Step 3: Controller Synthesis for Reachability Specification")
reach_controller = reach_controller(control_delta,controller_syn)
# savemat("path",{'reach_controller':reach_controller})
print("--- Total Reachability Controller Compute Time %s seconds --- \n" % (time.time() - start_time))