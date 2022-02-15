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

speed=300

compliance_slope= 8                #7 values (2,4,8,16,32,64,128)
compliance_margin=25                # 0-255
torque_limit=400                       #0-1023

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


def read_location():
   
    
    for i in range(10):
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

        print("[ID:%03d]   PresPos:%03d\t[ID:%03d]  PresPos:%03d\t[ID:%03d]   PresPos:%03d" % (DXL1_ID, dxl1_present_position, DXL2_ID, dxl2_present_position,DXL3_ID, dxl3_present_position))

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

read_location()


# Close port
portHandler.closePort()