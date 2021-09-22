import numpy as np
from numpy import linalg
from numpy.core.defchararray import array
p=[3,4,5]
small=1e-6
alpha=[0,120,240]  #in degrees
L_1= np.array([[1,2,3],[2,3,4],[2,5,6]]) #length_vector of proximal links
L_2= np.array([[1,2,3],[2,3,4],[2,5,6]]) #length_vector of distal links

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
