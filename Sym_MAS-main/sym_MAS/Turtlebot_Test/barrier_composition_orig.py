import numpy as np
import numba as nb
import time

from scipy.io import loadmat, savemat
from scipy.sparse import csr_matrix
from scipy import sparse
from math import pi, sqrt
from sys import exit

bound_x = np.array([0, 5])
bound_y = np.array([0, 5])
bound_theta = np.array([-pi, pi])
bound = np.array([bound_x, bound_y, bound_theta])

bound_v = np.array([-1, 1])
bound_w = np.array([-2, 2])
bound_u = np.array([bound_v, bound_w])

print("bound_x:", bound_x.shape)
print("bound_y:", bound_y.shape)
print("bound_theta:", bound_theta.shape)
print("bound:", bound.shape)
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

print("d_x: ", d_x, " d_y: ", d_y, " d_theta :", d_theta)
print("d_states", d_states.size)

# Input discretization
n_v = 5
n_w = 5
n_modes = 5
d_v = (bound_v[1] - bound_v[0]) / (n_v)
d_w = (bound_w[1] - bound_w[0]) / (n_w)

u_values1 = np.arange(
    bound_u[0][0],
    bound_u[0][1] + (bound_u[0][1] - bound_u[0][0]) / (n_v),
    (bound_u[0][1] - bound_u[0][0]) / (n_v),
)
u_values2 = np.arange(
    bound_u[1][0],
    bound_u[1][1] + (bound_u[1][1] - bound_u[1][0]) / (n_w),
    (bound_u[1][1] - bound_u[1][0]) / (n_v),
)

print("u_values1", u_values1)
print("u_values2", u_values2)
print("d_x: ", d_x, " d_y: ", d_y, " d_theta :", d_theta)

#  Barrier Certificate = d-norm(x_i-x_j)-L
d = 1  # Distance to be maintained
L = 1  # Parameter for abstraction of barrier

# load Delta1 and Delta3
Delta1 = loadmat("D:/Git codes/Sym_MAS_Jeel_update/Sym_MAS/sym_MAS/Turtlebot_Test/controlled_Delta1")
print(Delta1["control_Delta1"].shape)
controlled_Delta1 = Delta1["control_Delta1"]
# controlled_Delta1 = controlled_Delta.toarray()

Delta2 = loadmat("D:/Git codes/Sym_MAS_Jeel_update/Sym_MAS/sym_MAS/Turtlebot_Test/controlled_Delta2")
print(Delta2["control_Delta2"].shape)
controlled_Delta2 = Delta2["control_Delta2"]
# controlled_Delta3 = controlled_Delta.toarray()

# Composition of Controlled System
Delta2 = csr_matrix(((n_x*n_y*n_theta*n_x*n_y*n_theta) * (n_v*n_w*n_v*n_w), n_x*n_y*n_theta*n_x*n_y*n_theta), dtype=bool)
Delta = Delta2.toarray()

# Delta = np.zeros(((n_x*n_y*n_theta*n_x*n_y*n_theta) * (n_v*n_w*n_v*n_w), n_x*n_y*n_theta*n_x*n_y*n_theta))
try:
    @nb.jit(nopython=True, nogil=True)
    def do_barrier_cal(Delta, controlled_Delta1, controlled_Delta3):
        for i1 in range(n_x):
            # put a waitbar to show progress
            print("Progress:- ", (i1 / n_x) * 100)
            for i2 in range(n_y):
                for i3 in range(n_theta):
                    for i4 in range(n_x):
                        for i5 in range(n_y):
                            for i6 in range(n_theta):
                                for h1 in range(n_v):
                                    for h2 in range(n_w):
                                        for h3 in range(n_v):
                                            for h4 in range(n_w):
                                                for ip1 in range(n_x):
                                                    for ip2 in range(n_y):
                                                        for ip3 in range(n_theta):
                                                            for ip4 in range(n_x):
                                                                for ip5 in range(n_y):
                                                                    # start_time_per =  time.time()
                                                                    for ip6 in range(n_theta):
                                                                        row_1 = ((i1)* n_y* n_theta* n_v* n_w+ (i2)* n_theta* n_v* n_w+ (i3)* n_v* n_w+ (h1) * n_w+ h2)
                                                                        col_1 = ((ip1)* n_y* n_theta+ (ip2)* n_theta+ ip3)

                                                                        if controlled_Delta1[row_1][col_1]:
                                                                            r1 = np.array([ip1,ip2])

                                                                        else:
                                                                            continue

                                                                        row_2 = ((i4)* n_y* n_theta* n_v* n_w+ (i5)* n_theta* n_v* n_w+ (i6)* n_v* n_w+ (h3) * n_w+ h4)
                                                                        col_2 = ((ip4)* n_y* n_theta+ (ip5)* n_theta+ ip6)
                                                                        if controlled_Delta3[row_2][col_2]:
                                                                            r2 = np.array([ip4,ip5])
                                                                        else:
                                                                            continue

                                                                        B_prime = (sqrt((r1[0] - r2[0])**2 + (r1[1] - r2[1])**2)-sqrt((i1 - i3)**2 + (i2 - i4)**2))
                                                                        if (B_prime >= -0.5*(sqrt((i1-i3)**2+(i2-i4)**2)-L-d)):
                                                                            row_3 = ((i1)* n_y* n_theta* n_x* n_y* n_theta* n_v* n_w* n_v* n_w+ (i2)* n_theta* n_x* n_y* n_theta* n_v* n_w* n_v* n_w+ (i3)* n_x* n_y* n_theta* n_v* n_w* n_v* n_w+ (i4)* n_y* n_theta* n_v* n_w* n_v* n_w+ (i5)* n_theta* n_v* n_w* n_v* n_w+ (i6)* n_v* n_w* n_v* n_w+ (h1)* n_w* n_v* n_w+ (h2)* n_v* n_w+ (h3)* n_w+ h4)
                                                                            col_3 = ((ip1)* n_y* n_theta* n_x* n_y* n_theta+ (ip2)* n_theta* n_x* n_y* n_theta+ (ip3)* n_x* n_y* n_theta+ (ip4)* n_y* n_theta+ (ip5)* n_theta+ ip6)
                                                                            Delta[row_3][col_3] = 1
        return Delta        
   
    def controller_computation(composed_delta):
        Cont = np.zeros((n_x*n_y*n_theta*n_x*n_y*n_theta,n_v*n_w*n_v*n_w),dtype=bool) #create the matrix structure (defined only with 0,1) 
        for i in range(n_x*n_y*n_theta*n_x*n_y*n_theta): #checking admissible modes for any state
            for h in range(n_v*n_w*n_v*n_w): #for any control mode
                succ = np.where(composed_delta[(i)*n_v*n_w*n_v*n_w+h][:] == 1)
                if succ==[]:
                    Cont[i][h] = 1
        return Cont
    
    # For Safety Specifications
    def safety_controller(Controller,composed_delta):
        Contp = np.zeros((n_x*n_y*n_theta*n_x*n_y*n_theta,n_v*n_w*n_v*n_w),dtype=bool) #create the matrix structure (defined only with 0,1) 
        iter=0
        while (not (np.array_equal(Contp, Controller))):
            iter += 1
            Contp = np.copy(Controller)
            for i in range(n_x*n_y*n_theta*n_x*n_y*n_theta):
                # print("Progress:- ", (i / (n_x*n_y*n_theta)) * 100)
                mode = np.where(Contp[i][:] == 1)
                if mode[0].size > 0:
                    for k in range(len(mode[0])):
                        val = mode[0][k]
                        succ = np.where(composed_delta[i*n_v*n_w*n_v*n_w + val][:] == 1)
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
        print(np.count_nonzero(Cont))
        while (not (np.array_equal(Contp, Cont))):

            iter += 1
            print("Iteration",iter)
            Contp = np.copy(Cont)
            for i in range(n_x*n_y*n_theta*n_x*n_y*n_theta):                
                mode = np.where(Cont_syn_orig[i][:] == 1)
                # print(mode)
                if mode[0].size > 0:
                    for k in range(len(mode[0])):
                        val = mode[0][k]
                        # print(mode[0][k])
                        succ = np.where(composed_delta[i*n_v*n_w*n_v*n_w + val][:] == 1)
                        if succ[0].size > 0:
                            # print(succ)    
                            for ip in range(len(succ[0])):
                                val1 = succ[0][ip]

                                if ~(np.isin(i,reach_states)):
                                    if ~(np.isin(val1,reach_states)):
                                        Cont[i][val] = 0
                                        break

                                    else:
                                        Cont[i][val] = 1
                                        reach_states = np.append(reach_states,i)

        print(reach_states)                                    
        print(np.count_nonzero(Cont))
        return Cont

    
    # Compute Automata ( reachable sets )
    print("Step 1: Computation of the Barrier Composition")
    start_time = time.time()
    composed_delta = do_barrier_cal(Delta, controlled_Delta1, controlled_Delta2)
    # savemat("path",{'composed_delta':composed_delta})
    print("--- Barrier Composition total Completion Time %s seconds ---" % (time.time() - start_time))

    print("Step 2: Computation of Composed Controller")
    start_time = time.time()
    composed_controller_syn = controller_computation(composed_delta)
    # savemat("path",{'composed_controller_syn':composed_controller_syn})

    # print("Step 2: Controller Synthesis for Safety Specification \n")
    # start_time = time.time()
    # composed_safe_controller = safety_controller(controller_syn,control_delta)
    # savemat("path",{'composed_safe_controller':composed_safe_controller})
    # print("--- Total Safety Controller Compute Time %s seconds --- \n" % (time.time() - start_time))

    print("Step 3: Controller Synthesis for Composed Reachability Specification")
    composed_reach_controller = reach_controller(composed_delta,composed_controller_syn)
    print(np.count_nonzero(composed_reach_controller))
    savemat("D:/Git codes/Sym_MAS_Jeel_update/Sym_MAS/sym_MAS/Turtlebot_Test/composed_reach_controller.mat",{'composed_reach_controller':composed_reach_controller})
    print("--- Total Reachability Controller Compute Time %s seconds --- \n" % (time.time() - start_time))

except KeyboardInterrupt:
    exit()