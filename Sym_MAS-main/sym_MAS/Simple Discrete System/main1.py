import numpy as np
import numba as nb
import time

from scipy.io import loadmat,savemat
from scipy.sparse import csr_matrix
from scipy import sparse
from math import pi, sqrt
from sys import exit

# FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), y(k+1)=y(k)+u_y(k)

bound_x = np.array([0, 4])
bound_y = np.array([0, 4])
bound = np.array([bound_x, bound_y])

bound_u = np.array([(-2,0),(-1,0) ,(1,0),(2,0),(0,-2),(0,-1),(0,1),(0,2)])
n_u = len(bound_u)
print("bound_x:", bound_x.shape)
print("bound_y:", bound_y.shape)
print("bound:", bound.shape)
print("bound_u:", bound_u.shape)

# Space discretization
# numbers of intervals
n_x = 5
n_y = 5

# size of the interval
d_x = (bound_x[1] - bound_x[0]) / (n_x)
d_y = (bound_y[1] - bound_y[0]) / (n_y)
d_states = np.array([d_x, d_y])

print("d_x: ", d_x, " d_y: ", d_y)
print("d_states", d_states)

#  Barrier Certificate = d-norm(x_i-x_j)-L
d = 1  # Distance to be maintained
L = 1  # Parameter for abstraction of barrier

# def uni2diff(u):
#     """Convert speed and angular rate to wheel speeds."""
#     v = u[0]
#     omega = u[1]
#     v_L = v - ELL / 2 * omega
#     v_R = v + ELL / 2 * omega
#     return np.array([v_L, v_R]

# Delta = csr_matrix(((n_x*n_y)*(n_x*n_y)*(n_u)*(n_u), (n_x*n_y)*(n_x*n_y)), dtype=bool)
# Delta = Delta2.tolil()
Delta = np.zeros(((n_x*n_y*n_u), (n_x*n_y)))
# @nb.jit(nopython=True, nogil=True)
def compute_succ(Delta):
    for i1 in range(n_x):
        print("Progress:- ", (i1 / n_x) * 100)
        for i2 in range(n_y):                        
            for h1 in range(n_u):
                xp1=i1+bound_u[h1][0]
                yp1=i2+bound_u[h1][1]
                if ((xp1>=bound_x[0]) and (yp1>=bound_y[0]) and (xp1<=bound_x[1]) and (yp1<=bound_y[1])):
                        # associate successors of extrema to extremal intervals
                        row = ((i1)*n_y*n_u + (i2)*n_u+ h1)
                        # print(Delta.shape)
                        # print("Row",row)
                        col = ((xp1)*n_y + (yp1))
                        # print("Column",col)
                        Delta[row,col]=1
    return Delta
# Controller Synthesis
def controller_synthesis(composed_delta):
        Cont = np.zeros((n_x*n_y,n_u),dtype=bool) #create the matrix structure (defined only with 0,1) 
        for i in range(n_x*n_y): #checking admissible modes for any state
            for h in range(n_u): #for any control mode
                succ = np.where(composed_delta[i*n_u+h][:] == 1)
                # print(succ)
                if succ[0].size > 0:
                    Cont[i][h] = 1
        return Cont

# For Safety Specifications
def safety_controller(Controller,composed_delta):
    Contp = np.zeros((n_x*n_y,n_u),dtype=bool) #create the matrix structure (defined only with 0,1) 
    iter=0
    while (not (np.array_equal(Contp, Controller))):
        iter += 1
        Contp = np.copy(Controller)
        for i in range(n_x*n_y):
            print("Progress:- ", (i / (n_x*n_y)) * 100)
            mode = np.where(Contp[i][:] == 1)
            if mode[0].size > 0:
                for k in range(len(mode[0])):
                    val = mode[0][k]
                    succ = np.where(composed_delta[i*n_u + val][:] == 1)
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

    Contp = np.zeros((n_x*n_y,n_u),dtype=bool) #create the matrix structure (defined only with 0,1)
    reach_states = 4*n_y+4

    iter=0
    while (not (np.array_equal(Contp, Cont))):
        iter += 1
        Contp = np.copy(Cont)
        for i in range(n_x*n_y):
            print("Progress:- ", (i / (n_x*n_y)) * 100)
            mode = np.where(Cont_syn_orig[i][:] == 1)
            if mode[0].size > 0:
                for k in range(len(mode[0])):
                    val = mode[0][k]
                    succ = np.where(composed_delta[i*n_u + val][:] == 1)
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
        print(np.count_nonzero(Cont))
    return Cont

def controller_transition(Delta,Cont):
    controlled_Delta1 = np.zeros((n_x*n_y*n_u,n_x*n_y),dtype=bool) 
    Delta1 = Delta.copy()
    for i in range(n_x*n_y):
        print("Progress:- ", (i / (n_x*n_y)) * 100)
        for h in range(n_u):
            if(Cont[i][h]):
                controlled_Delta1[i*n_u+h][:] = Delta1[i*n_u+h][:]
            else:
                break
    return controlled_Delta1

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

print("Step 4: Convert the Controller into a Transition System")
controlled_Delta1 = controller_transition(control_delta,reach_controller)
savemat("D:/Git codes/Sym_MAS/sym_MAS/Simple Discrete System/controlled_Delta1.mat",{'controlled_Delta1':controlled_Delta1})