import numpy as np

n_x = 5
n_y = 5
n_theta = 5
rxr1 = np.array([4, 5])
ryr1 = np.array([4, 5])
rxr2 = np.array([0, 1])
ryr2 = np.array([0, 1])
reach_states=[]
for i in range(2):
        for j in range(2):
            for t1 in range(5):
                for k in range(2):
                    for l in range(2):
                        for t2 in range(5):
                            reach_states=np.append(reach_states,rxr1[i]*n_y*n_theta*n_x*n_y*n_theta+ryr1[j]*n_theta*n_x*n_y*n_theta+t1*n_x*n_y*n_theta+rxr2[k]*n_y*n_theta+ryr2[l]*n_theta+t2)

print(reach_states)