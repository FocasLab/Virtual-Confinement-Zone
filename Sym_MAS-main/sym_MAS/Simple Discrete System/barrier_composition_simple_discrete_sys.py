import numpy as np
import numba as nb
import time

from scipy.io import loadmat,savemat
from scipy.sparse import csr_matrix
from scipy import sparse
from math import pi, sqrt
from sys import exit

# FOR THE DYNAMICS x(k+1)=x(k)+u_x(k), y(k+1)=y(k)+u_y(k)

bound_x = np.array([1, 5])
bound_y = np.array([1, 5])
bound = np.array([bound_x, bound_y])

bound_u = np.array([(-2,0),(-1,0) ,(1,0),(2,0),(0,-2),(0,-1),(0,1),(0,2)])
n_u = len(bound_u)
print("bound_x:", bound_x.shape)
print("bound_y:", bound_y.shape)
print("bound:", bound.shape)
print("bound_u:", bound_u.shape)

# Space discretization
# numbers of intervals
n_x = 50
n_y = 50

# size of the interval
d_x = (bound_x[1] - bound_x[0]) / (n_x)
d_y = (bound_y[1] - bound_y[0]) / (n_y)
d_states = np.array([d_x, d_y])

print("d_x: ", d_x, " d_y: ", d_y)
print("d_states", d_states)

#  Barrier Certificate = d-norm(x_i-x_j)-L
d = 1  # Distance to be maintained
L = 1  # Parameter for abstraction of barrier

# load Delta1 and Delta3
Delta1 = loadmat("D:/Git codes/Sym_MAS/sym_MAS/Simple Discrete System/Controlled_System1")

print(Delta1["controlled_Delta1"].shape)
controlled_Delta = Delta1["controlled_Delta1"]
controlled_Delta1 = controlled_Delta.toarray()

Delta2 = loadmat("D:/Git codes/Sym_MAS/sym_MAS/Simple Discrete System/Controlled_System2")
print(Delta2["controlled_Delta2"].shape)
controlled_Delta = Delta2["controlled_Delta2"]
controlled_Delta2 = controlled_Delta.toarray()

# Composition of Controlled System
Delta = csr_matrix(((n_x*n_y*n_x*n_y*n_u*n_u), (n_x*n_y*n_x*n_y)), dtype=bool)
# Delta = Delta2.todense()
# Delta = np.zeros(((n_x*n_y*n_x*n_y*n_u*n_u), (n_x*n_y*n_x*n_y)))

try:
    # @nb.jit(nopython=True, nogil=T
    # rue)
    def do_barrier_cal(Delta, controlled_Delta1, controlled_Delta2):
        for i1 in range(n_x):
            # put a waitbar to show progress
            print("Progress:- ", (i1 / n_x) * 100)
            for i2 in range(n_y):
                for i3 in range(n_x):
                    for i4 in range(n_y):
                        for h1 in range(n_u):
                            for h2 in range(n_u):
                                for ip1 in range(n_x):
                                    for ip2 in range(n_y):
                                        for ip3 in range(n_x):
                                            for ip4 in range(n_y):
                                                if ((sqrt((i1 - i3)** 2 + (i2 - i4)**2)- L - d)>=0) :

                                                    row_1 = ((i1)*n_y*n_u+(i2)*n_u+h1)
                                                    col_1 = ((ip1)*n_y+ip2)

                                                    if controlled_Delta1[row_1][col_1]:
                                                        r1 = np.array([ip1,ip2])
                                                    else:
                                                        continue

                                                    row_2 = ((i3)*n_y*n_u+(i4)*n_u+h2)
                                                    col_2 = ((ip3)*n_y+ip4)

                                                    if controlled_Delta2[row_2][col_2]:
                                                        r2 = np.array([ip3,ip4])
                                                    else:
                                                        continue

                                                    B_prime = (sqrt((r1[0] - r2[0])**2 + (r1[1] - r2[1])**2)-sqrt((i1 - i3)**2 + (i2 - i4)**2))
                                                    if ( B_prime >= -0.5*(sqrt((i1-i3)**2+(i2-i4)**2)-L-d) ):
                                                        row_3 = ((i1)*n_y*n_x*n_y*n_u*n_u + (i2)*n_x*n_y*n_u*n_u + (i3)*n_y*n_u*n_u + (i4)*n_u*n_u + (h1)*n_u + h2)
                                                        col_3 = ((ip1)*n_y*n_x*n_y+(ip2)*n_x*n_y+(ip3)*n_y + ip4)
                                                        Delta[row_3][col_3] = 1

    start_time = time.time()
    do_barrier_cal(Delta, controlled_Delta1, controlled_Delta2)
    print("--- Barrier Composition total Completion Time %s seconds ---" % (time.time() - start_time))
    print(np.count_nonzero(Delta))

    # Controller Synthesis
    def controller_synthesis(composed_delta):
        Cont = np.zeros((n_x*n_y*n_x*n_y,n_u*n_u),dtype=bool) #create the matrix structure (defined only with 0,1) 
        for i in range(n_x*n_y*n_x*n_y): #checking admissible modes for any state
            for h in range(n_u*n_u): #for any control mode
                succ = np.where(composed_delta[i*n_u*n_u+h][:] == 1)
                # print(succ)
                if succ[0].size > 0:
                    Cont[i][h] = 1
        print(np.count_nonzero(Cont))
        return Cont
    
    start_time = time.time()
    controller_syn = controller_synthesis(Delta)
    print("--- Total Controller Synthesis Time %s seconds ---" % (time.time() - start_time))
    # savemat("E:/New folder/Sym_MAS/sym_MAS/Simple Discrete System/controllersyn_py.mat",{'controller_syn':controller_syn})

    # For Safety Specifications
    def safety_controller(Controller,composed_delta):
        Contp = np.zeros((n_x*n_y*n_x*n_y,n_u*n_u),dtype=bool) #create the matrix structure (defined only with 0,1) 
        iter=0
        while (not (np.array_equal(Contp, Controller))):
            iter += 1
            print("Iteration",iter)
            Contp = np.copy(Controller)
            for i in range(n_x*n_y*n_x*n_y):
                mode = np.where(Contp[i][:] == 1)
                if mode[0].size > 0:
                    for k in range(len(mode[0])):
                        val = mode[0][k]
                        succ = np.where(composed_delta[i*n_u*n_u + val][:] == 1)
                        if succ[0].size > 0:
                            for ip in range(len(succ[0])):
                                val1 = succ[0][ip]
                                if ~np.any(Contp[val1] == 1):
                                    Controller[i][val] = 0
                                    break
        print(np.count_nonzero(Contp))
        print(np.count_nonzero(Controller))
        return Controller

    # start_time = time.time()
    # safe_controller = safety_controller(controller,Delta)
    # print("--- Total Safety Controller Compute Time %s seconds ---" % (time.time() - start_time))

    # For Reachability
    def reach_controller(composed_delta,Cont_syn):
        Cont_syn_orig = np.copy(Cont_syn)
        Cont = np.copy(Cont_syn)

        Contp = np.zeros((n_x*n_y*n_x*n_y,n_u*n_u),dtype=bool) #create the matrix structure (defined only with 0,1)
        reach_states = np.array([(5-1)*n_y*n_x*n_y + (5-1)*n_x*n_y + (1-1)*n_y + 1-1])
        reach_states_target = np.copy(reach_states)
        
        iter=0
        print(np.count_nonzero(Cont))
        while (not (np.array_equal(Contp, Cont))):

            iter += 1
            print("Iteration",iter)
            Contp = np.copy(Cont)
            for i in range(n_x*n_y*n_x*n_y):                
                mode = np.where(Cont_syn_orig[i][:] == 1)
                # print(mode)
                if mode[0].size > 0:
                    for k in range(len(mode[0])):
                        val = mode[0][k]
                        # print(mode[0][k])
                        succ = np.where(composed_delta[i*n_u*n_u + val][:] == 1)
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

        return Cont

    start_time = time.time()
    reach_controller = reach_controller(Delta,controller_syn)
    print("--- Total Reachability Controller Compute Time %s seconds ---" % (time.time() - start_time))
    # savemat("E:/New folder/Sym_MAS/sym_MAS/Simple Discrete System/Reach_py.mat",{'reach_controller':reach_controller})
    
except KeyboardInterrupt:
    exit()