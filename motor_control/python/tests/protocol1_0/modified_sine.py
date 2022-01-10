import numpy as np
import matplotlib.pyplot as plt



p1=np.array([0,0,0.2])
p2=np.array([0.05,0.09,0.15])
S=np.linalg.norm(p1-p2)

max_vel=40
Cv=69.47
T=np.cbrt(Cv*S/max_vel)

time_stamp=np.arange(0,T,T/100)

St=[]
Trajectory_points=[]
for t in time_stamp:
    ratio=t/T
    if ratio< 1/8:
        St.append(S*((np.pi*ratio/(4+np.pi))- (np.sin(4*ratio*np.pi)/(4*(4+np.pi)))))
    elif ratio<7/8:
        St.append(S*(2/(4+np.pi)+(np.pi*ratio/(4+np.pi))- (9*np.sin(4*ratio*np.pi/3 +np.pi/3)/(4*(4+np.pi)))))
    else:
        St.append(S*(4/(4+np.pi)+(np.pi*ratio/(4+np.pi))- (np.sin(4*ratio*np.pi)/(4*(4+np.pi)))))
    Trajectory_points.append(list(p1+St[-1]*(-p1+p2)/S))

print(Trajectory_points)
plt.plot(time_stamp,Trajectory_points)
plt.ylabel('trajectory_points')
plt.show()