#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Modified by Eunchan Park (http://YouTube.com/c/EunChanPark) for AX12
# Author: Ryu Woon Jung (Leon)
#
# *********     Ping Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 1.0
# This example is tested with a Dynamixel AX-12A, and an USB2DYNAMIXEL in Windows PC
# Be sure that Dynamixel AX properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 2000000)
#

import os
from time import sleep
import numpy as np
import RPi.GPIO as GPIO

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_PRESENT_POSITION   = 36

ADDR_Moving_speed = 32

ADDR_Compliance_slope_CW=28
ADDR_Compliance_slope_CCW=29

ADDR_Compliance_margin_CW=26
ADDR_Compliance_margin_CCW=27

ADDR_Torque_LImit=34

# Data Byte Length
LEN_AX_GOAL_POSITION       = 2
LEN_AX_PRESENT_POSITION    = 2

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 0                  # Dynamixel#1 ID : 3


BAUDRATE                    = 2000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = "/dev/ttyUSB0"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
#DXL_MINIMUM_POSITION_VALUE  = 600           # Dynamixel will rotate between this value
#DXL_MAXIMUM_POSITION_VALUE  = 700            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

DXL_MINIMUM_POSITION_VALUE_1  = 600
DXL_MAXIMUM_POSITION_VALUE_1  = 700

DXL_MINIMUM_POSITION_VALUE_2  = 600
DXL_MAXIMUM_POSITION_VALUE_2  = 700

DXL_MINIMUM_POSITION_VALUE_3  = 600
DXL_MAXIMUM_POSITION_VALUE_3  = 700

DXL_MINIMUM_SPEED_VALUE  = 50           # Dynamixel will rotate between this value
#DXL_MAXIMUM_SPEED_VALUE  = 100            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 100                # Dynamixel moving status threshold

speed=600

compliance_slope= 4                #7 values (2,4,8,16,32,64,128)
compliance_margin=100                # 0-255
torque_limit=512                       #0-1023

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)



#lengths are in meters


L=0.160      #upper legs length
l=0.225     #lower legs parallelogram length
h=0.036      #lower legs parallelogram width
wb=0.130     #planar distance from {0} to near base side
sb=6*wb/np.sqrt(3)    #base equilateral triangle side
ub=2*wb     #planar distance from {0} to a base vertex

up=0.03764      #planar distance from {P} to a platform vertex
sp=up*np.sqrt(3)      #platform equilateral triangle side
wp=up/2      #planar distance from {P} to near platform side

a=wb-up
b=sp/2 - wb* (np.sqrt(3)/2)
c=wp - wb/2






suction=7
vcc=8



GPIO.setmode(GPIO.BCM)

GPIO.setup(vcc,GPIO.OUT)
GPIO.output(vcc,GPIO.HIGH)
GPIO.setup(suction,GPIO.OUT)
GPIO.output(suction,GPIO.LOW)



def change_gripper_state(state=False):
    try:
        #for i in range(2):
        GPIO.output(suction,state)
        print("gripper state changed")
        #time.sleep(3)
        #GPIO.output(suction,False)
        #time.sleep(3)
    except:
        raise

def InvKin(coordinates=[0,0,0.2]):
    #x,y,z=map(float,input("enter end-effector coordinates x y z\
    #    : ").split())
    z=-0.2
    x,y=[0,0]
    #coordinates=list(np.array(list(map(float,input("enter coordinates in mm: ").split())))/1000)
    
    if len(coordinates)==3:
        x,y,z=coordinates
        z=-z
    else:
        x,y=coordinates
    E=[2*L*(y+a), -L*((x+b)*np.sqrt(3)+y+c), L*((x-b)*np.sqrt(3)-y-c)]
    F=[2*z*L, 2*z*L, 2*z*L]
    G=[x**2+y**2+z**2+a**2+L**2+2*y*a-l**2, x**2+y**2+z**2+b**2+c**2+L**2+2*(x*b+y*c)-l**2,\
        x**2+y**2+z**2+b**2+c**2+L**2+2*(y*c-x*b)-l**2]

    theta=[]
    offset=0 #12.78434867110548
    for i in range(3):
        
        t_val=np.roots([G[i]-E[i], 2*F[i], G[i]+E[i]])
        t=np.array([2*np.arctan(i)*180/np.pi for i in t_val if np.isreal(i)==True ])
        if len(t)!=0:
            t_indx=np.argmin(np.absolute(t))
            theta.append(t[t_indx]+offset)

    if len(theta)==0:
        #rospy.loginfo("no solutions found!")
        if __name__=="__main__":
            print("no solutions found!")
    else:
        #rospy.loginfo(f"\n\nThe three angles of the servos\
        #from their horizontal position are {theta}")
        
        #if __name__=="__main__":
            
        return list(np.array((150+np.array(theta))*1023/300).astype(int))



def move_to_location(coordi=InvKin([0,0,0.180])):
    
    #coordinates=list(np.array(list(map(float,input("enter coordinates in mm: ").split())))/1000)
    #InvKin(coordi)
    index = 1
    dxl_goal_position_1 = [DXL_MINIMUM_POSITION_VALUE_1, coordi[0]]         # Goal position_1
    dxl_goal_position_2 = [DXL_MINIMUM_POSITION_VALUE_2, coordi[1]]         # Goal position_2
    dxl_goal_position_3 = [DXL_MINIMUM_POSITION_VALUE_3, coordi[2]]         # Goal position_3
    

    #print("target2", DXL_MAXIMUM_POSITION_VALUE_2)
    # Allocate goal position value into byte array
    # param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]
    # because AX12's goal posiion is only 2 bytes. only needed to split them once.
    param_goal_position_1 = [DXL_LOBYTE(dxl_goal_position_1[index]), DXL_HIBYTE(dxl_goal_position_1[index])]
    param_goal_position_2 = [DXL_LOBYTE(dxl_goal_position_2[index]), DXL_HIBYTE(dxl_goal_position_2[index])]
    param_goal_position_3 = [DXL_LOBYTE(dxl_goal_position_3[index]), DXL_HIBYTE(dxl_goal_position_3[index])]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result_1 = groupSyncWrite.addParam(DXL1_ID, param_goal_position_1)

    if dxl_addparam_result_1 != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result_2 = groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
    if dxl_addparam_result_2 != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()

     # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result_3 = groupSyncWrite.addParam(DXL3_ID, param_goal_position_3)
    if dxl_addparam_result_3 != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    
    while 1:
        # Read Dynamixel#1 present position
        dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Read Dynamixel#2 present position
        dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Read Dynamixel#2 present position
        dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position_1[index], dxl1_present_position, DXL2_ID, dxl_goal_position_2[index], dxl2_present_position,DXL3_ID,dxl_goal_position_3[index], dxl3_present_position))

        if not ((abs(dxl_goal_position_1[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position_2[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)and (abs(dxl_goal_position_3[index] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
            break
    #sleep(0.01)'''




# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)


# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)




#set torque limit
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_Torque_LImit, torque_limit)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_Torque_LImit, torque_limit)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_Torque_LImit, torque_limit)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#set speed
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_Moving_speed, speed)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#sleep(0.25)

dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_Moving_speed, speed)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#sleep(0.25)

dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_Moving_speed, speed)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))
#sleep(0.25)





#set compliance margin CW for DXL1_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_Compliance_margin_CW, compliance_margin)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#set compliance margin CCW for DXL1_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_Compliance_margin_CCW, compliance_margin)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))


#set compliance margin CW for DXL2_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_Compliance_margin_CW, compliance_margin)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#set compliance margin CCW for DXL2_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_Compliance_margin_CCW, compliance_margin)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))


#set compliance margin CW for DXL3_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_Compliance_margin_CW, compliance_margin)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#set compliance margin CCW for DXL3_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_Compliance_margin_CCW, compliance_margin)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))




#set compliance slope CW for DXL1_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_Compliance_slope_CW, compliance_slope)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#set compliance slope CCW for DXL1_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_Compliance_slope_CCW, compliance_slope)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))


#set compliance slope CW for DXL2_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_Compliance_slope_CW, compliance_slope)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#set compliance slope CCW for DXL2_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_Compliance_slope_CCW, compliance_slope)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))


#set compliance slope CW for DXL3_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_Compliance_slope_CW, compliance_slope)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))

#set compliance slope CCW for DXL3_ID
dxl_com_result, dxl__error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_Compliance_slope_CCW, compliance_slope)
if dxl_com_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_com_result))
elif dxl__error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl__error))









#r=0.06
points=[[0.020,0,0.15],[0.0,0.03,0.15],[0.04,-0.03,0.15]]
#points=[[r*np.cos(theta*np.pi/180),r*np.sin(theta*np.pi/180),0.2] for theta in range(0,360,1)]
coordinates=[]
for i in points:
    ikin_result=InvKin(i)
    coordinates.append(ikin_result)

sleep(1)

move_to_location()
grip_status=True
for targets in coordinates:
    move_to_location(targets)
    sleep(1)
    change_gripper_state(grip_status)
    grip_status=not grip_status
    sleep(2)

move_to_location()
change_gripper_state()
    
GPIO.cleanup()


# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()