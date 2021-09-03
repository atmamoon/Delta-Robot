import numpy as np

accep_p= np.array([1,2,3])  #acceleration of end-effector
vel_p= np.array([1,2,3])    #velocity of end-effector

L= np.array([[1,2,3],[2,3,4]]) #lengths of links, [[proximal_link, distal_link]]

alpha=[0,120,240] #angles between legs from top view w.r.t global_frame

phi=np.array([[2,5,6],[2,5,6],[2,5,6]]) #angles of links as per diagream [[p11,p12,p12]]


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
vel_2 = [vel_p - np.cross(omega_2[i],L[1,i])/2] #velocity of COG of distal link

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


