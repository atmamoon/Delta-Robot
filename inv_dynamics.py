import numpy as np
from numpy import linalg
from numpy.core.numeric import cross
from numpy.linalg.linalg import norm

#all units are in SI

g = np.array([0,0,9.8]) #acceleration due to gravity
accep_p= np.array([1,2,3])  #acceleration of end-effector
vel_p= np.array([1,2,3])    #velocity of end-effector

#diameters of distal links, considered as hollow rods
d2_outer=3
d2_inner=3 


L= np.array([[1,2,3],[2,3,4]]) #length_vector of links, [[proximal_link, distal_link]]

alpha=[0,120,240] #angles between legs from top view w.r.t global_frame

phi=np.array([[2,5,6],[2,5,6],[2,5,6]]) #angles of links as per diagream [[p11,p12,p12]]

m2= [3,3,3] #mass of distal links
m1= [3,3,3] #masses of proximal links
mp= 3 #mass of end-effector and load

#dimesions in end-effector
b= [4,4,4,4,4,4] #center of end effector to center of rod-end bearing sphere
rPS= 5 #from center to center of gravity


c=[[[3,3,2],[3,3,5]],[[3,3,2],[3,3,5]],[[3,3,2],[3,3,5]]] #length vectors from axis of proximal link to centre of rod end bearings

def Rotation_matrix(a):
    #input- angle in degrees
    #global frame w.r.t local frame (Tio)
    a=a*np.pi/180
    return np.array([[np.cos(a),np.sin(a),0],[-np.sin(a),np.cos(a),0],\
    [0,0,1]])

#axis of rotation of proximal links w.r.t global frame
sA= [np.dot(np.transpose(Rotation_matrix(alpha[0])),np.array[0,1,0]),\
    np.dot(np.transpose(Rotation_matrix(alpha[1])),np.array[0,1,0]),\
    np.dot(np.transpose(Rotation_matrix(alpha[2])),np.array[0,1,0])]

omega_1 = [(np.dot(np.transpose(L[1,i])/(np.dot(np.cross(sA[i],L[0,i]),L[1,i])),vel_p))* sA[i] \
    for i in range(3)]

omega_2 = [(np.cross(L[1,i],(vel_p - np.cross(omega_1,L[0,i]))))/ (np.linalg.norm(L[1,1])) \ 
    for i in range(3)]

vel_1 = [np.cross(omega_1[i],L[0,i])/2 for i in range(3)] #velocity of COG of proximal link
vel_2 = [vel_p - np.cross(omega_2[i],L[1,i])/2  for i in range(3)] #velocity of COG of distal link

#angular acceleration of actuated proximal link
phi_dot_dot_1= [np.dot(accep_p - np.cross(omega_1[i],np.cross(omega_1[i],L[0,i])) - np.cross(omega_2[i],\
    np.cross(omega_2[i],L[1,i])), L[1,i]) / (np.dot(np.cross(sA[i],L[0,i]),L[1,i]))  for i in range(3)]  
omega_dot_1 = [phi_dot_dot_1[i] * sA[i] for i in range(3)]


#angular acceleration of distal link
omega_dot_2= [np.cross(L[1,i],(accep_p - np.cross(omega_1[i],np.cross(omega_1[i],L[0,i])) - np.cross(omega_dot_1[i],L[0,i])- \
    np.cross(omega_2[i],np.cross(omega_2[i],L[1,i])))) * np.linalg.norm(L[1,0])  for i in range(3)]


#acceleration of COG of proximal and distal links
accel_1 = [np.cross(omega_dot_1,L[0,i]/2) + np.cross(omega_1[i],np.cross(omega_1[i],L[0,i]/2))  for i in range(3)]
accel_2 = [2*accel_1[i] + np.cross(omega_2[i],np.cross(omega_2[i],L[1,i]/2))+ np.cross(omega_dot_2[i],L[1,i]/2)  for i in range(3)]



#Rotation matrix of frame attached to distal link w.r.t local frame
Q2 = [[L[1,i]/np.linalg.norm(L[1,i]), np.cross(np.transpose([1,0,0]),L[1,i])/ np.linalg.norm(np.cross(np.transpose([1,0,0]),L[1,i])), \
    np.cross(L[1,i], np.cross(np.transpose[1,0,0],L[1,i]))/ np.linalg.norm(np.cross(L[1,i], np.cross(np.transpose[1,0,0],L[1,i])))]  for i in range(3)]

#Inertia matrix of distal link
I_2_local = [[d2_outer**2 + d2_inner**2,0,0],[0, np.linalg.norm(L[1,0])**2/12,0],[0,0,np.linalg.norm(L[1,0])**2/12]] #Inertia matrix w.r.t axis attached to distal link
I2 = [np.dot(Q2[i], np.dot(I_2_local,np.transpose(Q2[i]))) for i in range(3)] #Inertia matrix w.r.t local frame


#Rotation matrix of proximal link w.r.t global frame
Q1 = [np.dot(Rotation_matrix(alpha[i]), [[np.cos(phi[i,0]),0,-np.sin(phi[i,0])],[0,1,0],[np.sin(phi[i,0]),0,np.cos(phi[i,0])]]) \
     for i in range(3)]

#Inertia matrix of proximal link
I_1_local= []
I1 = [np.dot(Q1[i], np.dot(I_1_local,np.transpose(Q1[i]))) for i in range(3)]


#Tangential force on distal link
Ft= [(np.cross(np.cross(L[1,i], m2[i]* accel_2[i])/2 - np.cross(L[1,i], m2[i]* g)/2 + np.dot(I2[i],omega_dot_2[i]) + np.cross(omega_2[i],np.dot(I2[i],omega_2))),L[1,i])/(np.linalg.norm(L[1,i])**2)\
     for i in range(3)]


Jint= [[L[1,i//2]/np.linalg.norm(L[1,i//2]) for i in range(6)],[np.cross(b[i],L[1,i//2])/np.linalg.norm(L[1,i//2]) for i in range(6)]]
Fext = [[mp*(accep_p - g) + np.sum(Ft,axis=1)],[np.cross(rPS, (accep_p-g))*mp+np.sum([np.cross(b[i],Ft[i]) for i in range(6)])]]


F = np.dot(np.linalg.pinv(Jint),Fext)

Fo = [-Ft[i] + np.linalg.norm(F[i])* L[1,i//2]/ np.linalg.norm(L[1,i//2]) + m2* (accel_2[i//2] - g)  for i in range(6)]

Mo = [np.sum([np.cross(c[i,k],Fo[2*i+k]) for k in range(2)]) +np.cross(L[0,i],(accel_1[i] - g))*m1/2 + np.dot(I1[i],omega_dot_1[i]) + np.cross(omega_1[i],np.dot(I1[i],omega_1[i]))\
     for i in range(3)]


Mo_local = [np.dot(np.transpose(Q1[i]),Mo[i]) for i in range(3)] #Moment w.r.t local frame
