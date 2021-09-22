import numpy as np
from numpy import linalg
from numpy.core.numeric import cross
from numpy.linalg.linalg import norm

#all units are in SI

g = np.array([0,0,9.8]) #acceleration due to gravity
accep_p= np.array([1,2,3])  #acceleration of end-effector
vel_p= np.array([1,2,3])    #velocity of end-effector

p=[3,4,5]

#diameters of distal links, considered as hollow rods
d2_outer=0.03
d2_inner=0.02

small= 1e-6 #small value to avoid division by zero

L_1= np.array([[1,2,3],[2,3,4],[2,5,6]]) #length_vector of proximal links
L_2= np.array([[1,2,3],[2,3,4],[2,5,6]]) #length_vector of distal links

alpha=[0,120,240] #angles between legs from top view w.r.t global_frame



Tio=np.array([[[np.cos(alpha[i]*np.pi/180),np.sin(alpha[i]*np.pi/180),0],[-np.sin(alpha[i]*np.pi/180),np.cos(alpha[i]*np.pi/180),0],\
    [0,0,1]] for i in range(3)])
b= [[3,2,5],[3,2,5],[3,2,5],[3,2,5],[3,2,5],[3,2,5]]
a= [[2,5,0],[2,5,0],[2,5,0]]
ri_i= -np.linalg.norm(a[0])+np.linalg.norm(b[0])+np.array([np.dot(Tio[i],p) for i in range(3)])

phi3i= [np.arccos((-p[0]*np.sin(alpha[i]*np.pi/180)+p[1]*np.cos(alpha[i]*np.pi/180))/(np.linalg.norm(L_2[i]))) for i in range(3)]   #in radians
print(phi3i)

phi2i= [np.arccos((np.linalg.norm(ri_i[i])**2 - np.linalg.norm(L_1[i])**2- np.linalg.norm(L_2[i])**2)/(2*np.linalg.norm(L_2[i])*np.linalg.norm(L_1[i])*np.sin(phi3i[i])))\
    for i in range(3)]  #in radians
print("phi2i",phi2i)
phi1i= [np.arctan(-(-np.linalg.norm(L_1[i])*ri_i[i,2] -np.linalg.norm(L_2[i])*np.sin(phi3i[i])*np.cos(phi2i[i])*ri_i[i,2]) + np.linalg.norm(L_2[i])*np.sin(phi3i[i])*np.sin(phi2i[i])\
    *ri_i[i,0])/(np.linalg.norm(L_1[i])*ri_i[i,0]+np.linalg.norm(L_2[i])*np.sin(phi3i[i])*np.sin(phi2i[i])*ri_i[i,2]+np.linalg.norm(L_2[i])*np.sin(phi3i[i])*np.cos(phi2i[i])*ri_i[i,0]) for i in range(3)] #in radians
print("phi1i",phi1i)

phi=np.array([phi1i,phi2i,phi3i])

#phi=np.array([[2,5,6],[2,5,6],[2,5,6]]) #angles of links as per diagream [[p11,p12,p12]]

m2= [3,3,3] #mass of distal links
m1= [3,3,3] #masses of proximal links
mp= 3 #mass of end-effector and load

#dimesions in end-effector
b= [[3,2,5],[3,2,5],[3,2,5],[3,2,5],[3,2,5],[3,2,5]] #center of end effector to center of rod-end bearing sphere
rPS= [5,3,5] #from center of endeffector to center of gravity


c=[[[3,3,2],[3,3,5]],[[3,3,2],[3,3,5]],[[3,3,2],[3,3,5]]] #length vectors from axis of proximal link to centre of rod end bearings

def Rotation_matrix(a):
    #input- angle in degrees
    #global frame w.r.t local frame (Tio)
    a=a*np.pi/180
    return np.array([[np.cos(a),np.sin(a),0],[-np.sin(a),np.cos(a),0],\
    [0,0,1]])

#axis of rotation of proximal links w.r.t global frame
"""sA= [np.dot(np.transpose(Rotation_matrix(alpha[0])),np.array([0,1,0]),\
    np.dot(np.transpose(Rotation_matrix(alpha[1])),np.array([0,1,0])),\
    np.dot(np.transpose(Rotation_matrix(alpha[2])),np.array([0,1,0]))]  """
sA= [np.dot(np.transpose(Rotation_matrix(alpha[i])),np.array([0,1,0])) \
    for i in range(3)]
print(sA[0])
omega_1 = [(np.dot(np.transpose(L_2[i])/(small + np.dot(np.cross(sA[i],L_1[i]),L_2[i])),vel_p))* sA[i] \
    for i in range(3)]

omega_2 = [(np.cross(L_2[i],(vel_p - np.cross(omega_1[i],L_1[i]))))/ (np.linalg.norm(L_2[1])) \
    for i in range(3)]
print("omega_2",np.shape(omega_1))
print(omega_2)

vel_1 = [np.cross(omega_1[i],L_1[i])/2 for i in range(3)] #velocity of COG of proximal link
vel_2 = [vel_p - np.cross(omega_2[i],L_2[i])/2  for i in range(3)] #velocity of COG of distal link

#angular acceleration of actuated proximal link
phi_dot_dot_1= [np.dot(accep_p - np.cross(omega_1[i],np.cross(omega_1[i],L_1[i])) - np.cross(omega_2[i],\
    np.cross(omega_2[i],L_2[i])), L_2[i]) / (small + np.dot(np.cross(sA[i],L_1[i]),L_2[i]))  for i in range(3)]  
omega_dot_1 = [phi_dot_dot_1[i] * sA[i] for i in range(3)]


#angular acceleration of distal link
omega_dot_2= [np.cross(L_2[i],(accep_p - np.cross(omega_1[i],np.cross(omega_1[i],L_1[i])) - np.cross(omega_dot_1[i],L_1[i])- \
    np.cross(omega_2[i],np.cross(omega_2[i],L_2[i])))) * np.linalg.norm(L_2[0])  for i in range(3)]


#acceleration of COG of proximal and distal links
accel_1 = [np.cross(omega_dot_1[i],L_1[i]/2) + np.cross(omega_1[i],np.cross(omega_1[i],L_1[i]/2))  for i in range(3)]
accel_2 = [2*accel_1[i] + np.cross(omega_2[i],np.cross(omega_2[i],L_2[i]/2))+ np.cross(omega_dot_2[i],L_2[i]/2)  for i in range(3)]
print("accel_1",np.shape(accel_1))
print("accel_1",np.shape(accel_2))


#Rotation matrix of frame attached to distal link w.r.t local frame
Q2 = [[L_2[i]/np.linalg.norm(L_2[i]), np.cross(np.transpose([1,0,0]),L_2[i])/ np.linalg.norm(np.cross(np.transpose([1,0,0]),L_2[i])), \
    np.cross(L_2[i], np.cross(np.transpose([1,0,0]),L_2[i]))/ np.linalg.norm(np.cross(L_2[i], np.cross(np.transpose([1,0,0]),L_2[i])))]  for i in range(3)]

#Inertia matrix of distal link
I_2_local = [[d2_outer**2 + d2_inner**2,0,0],[0, np.linalg.norm(L_2[0])**2/12,0],[0,0,np.linalg.norm(L_2[0])**2/12]] #Inertia matrix w.r.t axis attached to distal link
I2 = [np.dot(Q2[i], np.dot(I_2_local,np.transpose(Q2[i]))) for i in range(3)] #Inertia matrix w.r.t local frame


#Rotation matrix of proximal link w.r.t global frame
Q1 = [np.dot(Rotation_matrix(alpha[i]), [[np.cos(phi[i,0]),0,-np.sin(phi[i,0])],[0,1,0],[np.sin(phi[i,0]),0,np.cos(phi[i,0])]]) \
     for i in range(3)]

#Inertia matrix of proximal link
I_1_local= I_2_local
I1 = [np.dot(Q1[i], np.dot(I_1_local,np.transpose(Q1[i]))) for i in range(3)]

#Tangential force on distal link
Ft= [((np.cross(np.cross(L_2[i], m2[i]* accel_2[i])/2 - np.cross(L_2[i], m2[i]* g)/2 + np.dot(I2[i],omega_dot_2[i]) + np.cross(omega_2[i],np.dot(I2[i],omega_2[i])),L_2[i]))/(np.linalg.norm(L_2[i])**2))\
     for i in range(3)]


Jint= [[L_2[i//2]/np.linalg.norm(L_2[i//2]) for i in range(6)],[np.cross(b[i],L_2[i//2])/np.linalg.norm(L_2[i//2]) for i in range(6)]]
Fext = [[mp*(accep_p - g) + np.sum(Ft,axis=0)],  [np.cross(rPS, (accep_p-g))*mp+np.sum([np.cross(b[i],Ft[i//2]) for i in range(6)],axis=0)]]
print("Ft",np.shape(Ft))
print("Jint",np.shape(Jint))
print("Fext",np.shape(Fext))
#print(Fext)
#F = np.dot(np.transpose(np.reciprocal(Jint)),Fext)
#F = np.dot(np.transpose(Jint),Fext)

Jint= np.array(Jint)
"""
JintT= Jint.transpose((1,0,2))
Jinv= np.array([np.linalg.inv(np.dot(JintT[:,:,i],Jint[:,:,i])) for i in range(3)])
F= np.dot(Jinv,np.dot(JintT,Fext))
"""
Fext= np.array(Fext)
Jinv= np.array([np.linalg.pinv(Jint[:,:,i]) for i in range(3)])
Jinv=Jinv.transpose((1,2,0))
print("Jinv",np.shape(Jinv))
#F= np.matmul(Jinv,Fext)
F= np.array([np.dot(Jinv[:,:,i],Fext[:,:,i]) for i in range(3)])
F= F.transpose((1,2,0))
print("F",np.shape(F))

Fo = [-Ft[i//2] + np.linalg.norm(F[i])* L_2[i//2]/ np.linalg.norm(L_2[i//2]) + m2* (accel_2[i//2] - g)  for i in range(6)]

Mo = [np.sum([np.cross(c[i][k],Fo[2*i+k]) for k in range(2)]) +np.cross(L_1[i],(accel_1[i] - g))*m1/2 + np.dot(I1[i],omega_dot_1[i]) + np.cross(omega_1[i],np.dot(I1[i],omega_1[i]))\
     for i in range(3)]


Mo_local = [np.dot(np.transpose(Q1[i]),Mo[i]) for i in range(3)] #Moment w.r.t local frame



print("Mo_local",np.around(Mo_local,decimals=3))