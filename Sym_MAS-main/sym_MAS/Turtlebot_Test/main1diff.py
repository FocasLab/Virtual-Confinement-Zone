import numpy as np
import time

from scipy.io import loadmat,savemat
from scipy.sparse import csr_matrix
from scipy import sparse
from math import pi, sqrt
from sys import exit

# ROS Imports
# import rospy
# from math import pow, atan2, sqrt
# from geometry_msgs.msg import Twist,Pose2D
# from phasespace_msgs.msg import Rigids
# from phasespace_msgs.msg import Markers
# import tf

# For Dynamics
class symbolic_cont():

    def __init__(self):
        self.tau = 2.1
        self.initial_state = [1,1,1]

        self.bound_x = np.array([0, 2])
        self.bound_y = np.array([0, 2])
        self.bound_theta = np.array([-pi, pi])
        self.bound = np.array([self.bound_x, self.bound_y, self.bound_theta])

        self.bound_v = np.array([-0.22, 0.22])
        self.bound_w = np.array([-0.11, 0.11])
        self.bound_u = np.array([self.bound_v,self.bound_w])

        # Space discretization
        # numbers of intervals
        self.n_x = 20
        self.n_y = 20
        self.n_theta = 20

        # size of the interval
        self.d_x = (self.bound_x[1] - self.bound_x[0]) / (self.n_x)
        self.d_y = (self.bound_y[1] - self.bound_y[0]) / (self.n_y)
        self.d_theta = (self.bound_theta[1] - self.bound_theta[0]) / (self.n_theta)
        self.d_states = np.array([self.d_x, self.d_y, self.d_theta])

        # Input discretization
        self.n_v = 11
        self.n_w = 11
        self.d_v = (self.bound_v[1] - self.bound_v[0]) / (self.n_v)
        self.d_w = (self.bound_w[1] - self.bound_w[0]) / (self.n_w)

        u_values1 = np.arange(self.bound_u[0][0],self.bound_u[0][1] + (self.bound_u[0][1] - self.bound_u[0][0]) / (self.n_v),(self.bound_u[0][1] - self.bound_u[0][0]) / (self.n_v))
        self.u_values1 = u_values1[0:12] 
        u_values2 = np.arange(self.bound_u[1][0],self.bound_u[1][1] + (self.bound_u[1][1] - self.bound_u[1][0]) / (self.n_w),(self.bound_u[1][1] - self.bound_u[1][0]) / (self.n_v))
        self.u_values2 = u_values2[0:12] 

        self.ELL = 0.2
        Delta2 = csr_matrix(((self.n_x*self.n_y*self.n_theta)*(self.n_v)*(self.n_w), (self.n_x*self.n_y*self.n_theta)), dtype=bool)
        self.Delta = Delta2.toarray()

    def diff_Successor(self,curr_state,input,time_step):
        # Set the track of the vehicle [m]
        # input = uni2diff(input)
        new_state = self.rk_four(self.diffdrive_f,curr_state,input,time_step)

        return new_state  

    def rk_four(self,f, x, u, T):
        """Fourth-order Runge-Kutta numerical integration."""
        k_1 = f(x, u)
        k_2 = f(x + T * k_1 / 2.0, u)
        k_3 = f(x + T * k_2 / 2.0, u)
        k_4 = f(x + T * k_3, u)
        x_new = x + T / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4)
        return x_new

    def diffdrive_f(self,x, u):
        """Differential drive kinematic vehicle model."""
        f = np.zeros(3)
        f[0] = (u[0]) * np.cos(x[2])
        f[1] = (u[0]) * np.sin(x[2])
        f[2] = (u[1])
        return f

    # @nb.jit(nopython=True, nogil=True)
    def compute_succ(self):
        Delta = self.Delta.copy()
        for i1 in range(self.n_x):
            print("Progress:- ", (i1 / self.n_x) * 100)
            for i2 in range(self.n_y):
                for i3 in range(self.n_theta):
                    bound_a1 = np.array([self.bound[:,0]]).T
                    bound_a2 = np.array([self.bound[:,0]]).T
                    a1 = np.multiply(np.array([i1, i2, i3+self.bound[2][0]/self.d_theta]),np.array([self.d_states])).T
                    a2 = np.multiply(np.array([i1+1, i2+1, i3+1+self.bound[2][0]/self.d_theta]),np.array([self.d_states])).T
                    a1 = bound_a1 + a1
                    a2 = bound_a2 + a2
                    x = np.concatenate((a1,a2),axis=1)
                    
                    for h1 in range(self.n_v):
                        for h2 in range(self.n_w):
                            u_values = np.array([self.u_values1[h1],self.u_values2[h2]])
                            # Successor
                            xp1 = self.diff_Successor(x[:,0],u_values,self.tau)
                            xp2 = self.diff_Successor(x[:,1],u_values,self.tau)

                            if ((all(np.greater_equal(xp1,self.bound[:,0]))) and (all(np.less_equal(xp2,self.bound[:,1])))):
                                # associate successors of extrema to extremal intervals
                                minsucc = np.floor((xp1-self.bound[:,0])/self.d_states)
                                maxsucc= np.ceil((xp2-self.bound[:,0])/self.d_states)
                                for ip1 in range(int(minsucc[0]),int(maxsucc[0])):
                                    for ip2 in range(int(minsucc[1]),int(maxsucc[1])):
                                        for ip3 in range(int(minsucc[2]),int(maxsucc[2])):
                                            row = ((i1)*self.n_y*self.n_theta*self.n_v*self.n_w + (i2)*self.n_theta*self.n_v*self.n_w+ (i3)*self.n_v*self.n_w + (h1)*self.n_w + h2)
                                            col = ((ip1)*self.n_y*self.n_theta + (ip2)*self.n_theta + ip3)
                                            Delta[row][col]=1
        return Delta

    # Controller Synthesis
    def controller_synthesis(self,composed_delta):
        Cont = np.zeros((self.n_x*self.n_y*self.n_theta,self.n_v*self.n_w),dtype=bool) #create the matrix structure (defined only with 0,1) 
        for i in range(self.n_x*self.n_y*self.n_theta): #checking admissible modes for any state
            print("Progress:- ", (i / (self.n_x*self.n_y*self.n_theta) * 100))
            for h in range(self.n_v*self.n_w): #for any control mode
                succ = np.where(composed_delta[i*self.n_v*self.n_w+h][:] == 1)
                if succ[0].size > 0:
                    Cont[i][h] = 1
        return Cont

    # For Safety Specifications
    def safety_controller(self,Controller,composed_delta):
        Contp = np.zeros((self.n_x*self.n_y*self.n_theta,self.n_v*self.n_w),dtype=bool) #create the matrix structure (defined only with 0,1) 
        iter=0
        while (not (np.array_equal(Contp, Controller))):
            iter += 1
            Contp = np.copy(Controller)
            for i in range(self.n_x*self.n_y*self.n_theta):
                print("Progress:- ", (i / (self.n_x*self.n_y*self.n_theta)) * 100)
                mode = np.where(Contp[i][:] == 1)
                if mode[0].size > 0:
                    for k in range(len(mode[0])):
                        val = mode[0][k]
                        succ = np.where(composed_delta[i*self.n_v*self.n_w + val][:] == 1)
                        if succ[0].size > 0:
                            for ip in range(len(succ[0])):
                                val1 = succ[0][ip]
                                if ~np.any(Contp[val1] == 1):
                                    Controller[i][val] = 0
                                    break
        return Controller

    # For Reachability
    def reach_controller(self,composed_delta,Cont_syn):
        Cont_syn_orig = np.copy(Cont_syn)
        Cont = np.copy(Cont_syn)

        Contp = np.zeros((self.n_x*self.n_y*self.n_theta,self.n_v*self.n_w),dtype=bool) #create the matrix structure (defined only with 0,1)
        rx = 0
        ry = 0
        rx1 = 1
        ry1 = 1
        reach_states = np.array([rx*self.n_y*self.n_theta+ry*self.n_theta+1-1 , rx*self.n_y*self.n_theta+ry*self.n_theta+2-1, rx*self.n_y*self.n_theta+ry*self.n_theta+3-1, rx*self.n_y*self.n_theta+ry*self.n_theta+4-1, rx*self.n_y*self.n_theta+ry*self.n_theta+5-1, (rx1)*self.n_y*self.n_theta+ry*self.n_theta+1-1, (rx1)*self.n_y*self.n_theta+ry*self.n_theta+2-1, (rx1)*self.n_y*self.n_theta+ry*self.n_theta+3-1, (rx1)*self.n_y*self.n_theta+ry*self.n_theta+4-1, (rx1)*self.n_y*self.n_theta+ry*self.n_theta+5-1, rx1*self.n_y*self.n_theta+ry1*self.n_theta+1-1, rx1*self.n_y*self.n_theta+ry1*self.n_theta+2-1, rx1*self.n_y*self.n_theta+ry1*self.n_theta+3-1, rx1*self.n_y*self.n_theta+ry1*self.n_theta+4-1, rx1*self.n_y*self.n_theta+ry1*self.n_theta+5-1, rx*self.n_y*self.n_theta+ry*self.n_theta+1-1, rx*self.n_y*self.n_theta+ry*self.n_theta+2-1, rx*self.n_y*self.n_theta+ry*self.n_theta+3-1, rx*self.n_y*self.n_theta+ry*self.n_theta+4-1, rx*self.n_y*self.n_theta+ry*self.n_theta+5-1])
        reach_states_target = np.copy(reach_states)
        
        iter=0
        while (not (np.array_equal(Contp, Cont))):
            iter += 1
            Contp = np.copy(Cont)
            for i in range(self.n_x*self.n_y*self.n_theta):
                print("Progress:- ", (i / (self.n_x*self.n_y*self.n_theta)) * 100)
                mode = np.where(Cont_syn_orig[i][:] == 1)
                if mode[0].size > 0:
                    for k in range(len(mode[0])):
                        val = mode[0][k]
                        succ = np.where(composed_delta[i*self.n_v*self.n_w + val][:] == 1)
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
        return Cont,reach_states_target

    def controller_transition(self,Delta,Cont):
        controlled_Delta1 = np.zeros((self.n_x*self.n_y*self.n_theta*self.n_v*self.n_w,self.n_x*self.n_y*self.n_theta),dtype=bool) 
        Delta1 = Delta.copy()
        for i in range(self.n_x*self.n_y*self.n_theta):
            print("Progress:- ", (i / (self.n_x*self.n_y*self.n_theta)) * 100)
            for h in range(self.n_v*self.n_w):
                if(Cont[i][h]):
                    controlled_Delta1[i*self.n_v*self.n_w+h][:] = Delta1[i*self.n_v*self.n_w+h][:]
                else:
                    break

        return controlled_Delta1

    def run_controller(self,control_delta,reach_control,reach_states_target,u_values1,u_values2,initial_state):
        init_x = initial_state[0]
        init_y = initial_state[1]
        init_theta = initial_state[2]

        curr_x = 0
        curr_y = 0
        curr_theta = 0
        pos = init_x*self.n_y*self.n_theta + init_y*self.n_theta + init_theta
        
        U_x = np.where(reach_control[pos][:] == 1)
        print(reach_states_target.shape)
        
        # Check if we are inside pos/target not compare values
        while( all(reach_states_target != pos)):
            # pos is current robot location
            pos = curr_x*self.n_y*self.n_theta + curr_y*self.n_theta + curr_theta
            U_x = np.where(reach_control[pos][:] == 1)
            if( U_x[0]% self.n_w == 0 ):
                w = self.n_w # index for angular cmd_vel
            else:
                w = U_x[0] % self.n_w 
            # U_x use first value 
            V = (U_x[0] - w)/self.n_w
            # index for linear cmd_vel
            linear_x = u_values1[V]
            angular_z = u_values2[w]

    def main(self):
        # Compute Automata ( reachable sets )
        print("Step 1: Computation of the abstraction's transition relation")
        start_time = time.time()
        control_delta = self.compute_succ()
        # savemat("path",{'control_delta':control_delta})
        print("--- Abstraction Transition total Completion Time %s seconds -- \n" % (time.time() - start_time))

        print("Step 2: Computation of Controller")
        start_time = time.time()
        controller_syn = self.controller_synthesis(control_delta)
        # savemat("path",{'controller_syn':controller_syn})
        print("--- Total Controller Synthesis Time %s seconds --- \n " % (time.time() - start_time))

        # print("Step 2: Controller Synthesis for Safety Specification \n")
        # start_time = time.time()
        # safe_controller = safety_controller(controller_syn,control_delta)
        # savemat("path",{'safe_controller':safe_controller})
        # print("--- Total Safety Controller Compute Time %s seconds --- \n" % (time.time() - start_time))

        print("Step 3: Controller Synthesis for Reachability Specification")
        start_time = time.time()
        reach_controller,reach_target = self.reach_controller(control_delta,controller_syn)
        # savemat("path",{'reach_controller':reach_controller})
        print("--- Total Reachability Controller Compute Time %s seconds --- \n" % (time.time() - start_time))

        print("Step 4: Converting the Controller into Transition System")
        start_time = time.time()
        control_Delta2 = self.controller_transition(control_delta,reach_controller)
        # np.savetxt("controlled_delta2.csv", control_Delta2, delimiter=",")
        # savemat("D:/Git codes/Sym_MAS_Jeel_update/Sym_MAS/sym_MAS/Turtlebot_Test/controlled_Delta2.mat",{'control_Delta2':control_Delta2})
        # savemat("E:/New folder/Sym_MAS/sym_MAS/Turtlebot_Test/controlled_Delta2.mat",{'control_Delta2':control_Delta2})
        print("--- Total Conversion Time %s seconds --- \n" % (time.time()-start_time))

        print("Step 5: Implementing the Controller into Real System")

        self.run_controller(control_delta,reach_controller,reach_target,self.u_values1,self.u_values2,self.initial_state)

if __name__=="__main__":
    symbolic_cont().main()