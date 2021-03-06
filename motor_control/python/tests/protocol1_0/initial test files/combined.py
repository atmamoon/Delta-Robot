
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

# Author: Ryu Woon Jung (Leon)

#
# *********     Protocol Combined Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 1.0 and 2.0
# This example is tested with a Dynamixel MX-28, a Dynamixel PRO 54-200 and an USB2DYNAMIXEL
# Be sure that properties of Dynamixel MX and PRO are already set as %% MX - ID : 1 / Baudnum : 34 (Baudrate : 57600) , PRO - ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

# Be aware that:
# This example configures two different control tables (especially, if it uses Dynamixel and Dynamixel PRO). It may modify critical Dynamixel parameter on the control table, if Dynamixels have wrong ID.
#
from time import sleep
import os

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

# Control table address for Dynamixel MX
ADDR_MX_TORQUE_ENABLE       = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_Moving_speed = 32
'''# Control table address for Dynamixel PRO
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132'''

# Protocol version
PROTOCOL_VERSION1           = 1.0               #  See which protocol version is used in the Dynamixel
#PROTOCOL_VERSION2           = 2.0

# Default setting
DXL1_ID                     = 0                 # Dynamixel#1 ID : 1
DXL2_ID                     = 1                 # Dynamixel#2 ID : 2
DXL3_ID                     = 2                 # Dynamixel#2 ID : 3
BAUDRATE                    = 2000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL1_MINIMUM_POSITION_VALUE = 50           # Dynamixel will rotate between this value
DXL1_MAXIMUM_POSITION_VALUE = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL2_MINIMUM_POSITION_VALUE = 50
DXL2_MAXIMUM_POSITION_VALUE = 1000
DXL3_MINIMUM_POSITION_VALUE = 50
DXL3_MAXIMUM_POSITION_VALUE = 1000
DXL1_MOVING_STATUS_THRESHOLD = 20                # Dynamixel MX moving status threshold
DXL2_MOVING_STATUS_THRESHOLD = 20                # Dynamixel PRO moving status threshold
DXL3_MOVING_STATUS_THRESHOLD = 20 

index = 0
dxl1_goal_position = [DXL1_MINIMUM_POSITION_VALUE, DXL1_MAXIMUM_POSITION_VALUE]         # Goal position of Dynamixel MX
dxl2_goal_position = [DXL2_MINIMUM_POSITION_VALUE, DXL2_MAXIMUM_POSITION_VALUE]         # Goal position of Dynamixel PRO
dxl3_goal_position = [DXL2_MINIMUM_POSITION_VALUE, DXL2_MAXIMUM_POSITION_VALUE] 

dxl1_speed= 300
dxl2_speed= 300
dxl3_speed= 300

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler1 = PacketHandler(PROTOCOL_VERSION1)
packetHandler2 = PacketHandler(PROTOCOL_VERSION1)
packetHandler3 = PacketHandler(PROTOCOL_VERSION1)

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
dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler1.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler2.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler3.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler3.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler3.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)


''''
dxl_speed_result, dxl_speed_error = packetHandler1.write4ByteTxRx(portHandler, DXL1_ID, ADDR_Moving_speed, dxl1_speed)
if dxl_speed_result != COMM_SUCCESS:
    print("%s" % packetHandler1.getTxRxResult(dxl_speed_result))
elif dxl_speed_error != 0:
    print("%s" % packetHandler1.getRxPacketError(dxl_speed_error))
sleep(0.25)

dxl_speed_result, dxl_speed_error = packetHandler2.write4ByteTxRx(portHandler, DXL2_ID, ADDR_Moving_speed, dxl2_speed)
if dxl_speed_result != COMM_SUCCESS:
    print("%s" % packetHandler1.getTxRxResult(dxl_speed_result))
elif dxl_speed_error != 0:
    print("%s" % packetHandler1.getRxPacketError(dxl_speed_error))

sleep(0.25)

dxl_speed_result, dxl_speed_error = packetHandler3.write4ByteTxRx(portHandler, DXL3_ID, ADDR_Moving_speed, dxl3_speed)
if dxl_speed_result != COMM_SUCCESS:
    print("%s" % packetHandler1.getTxRxResult(dxl_speed_result))
elif dxl_speed_error != 0:
    print("%s" % packetHandler1.getRxPacketError(dxl_speed_error))

sleep(0.25)
'''

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

   

    # Write Dynamixel#1 goal position
    dxl_comm_result, dxl_error = packetHandler1.write4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler1.getRxPacketError(dxl_error))

    time.sleep(2)

    # Write Dynamixel#2 goal position
    dxl_comm_result, dxl_error = packetHandler2.write4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler2.getRxPacketError(dxl_error))

    # Write Dynamixel#3 goal position
    dxl_comm_result, dxl_error = packetHandler3.write4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_GOAL_POSITION, dxl3_goal_position[index])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler3.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler3.getRxPacketError(dxl_error))

    while 1:
        # Read Dynamixel#1 present position
        dxl1_present_position, dxl_comm_result, dxl_error = packetHandler1.read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))

        # Read Dynamixel#2 present position
        dxl2_present_position, dxl_comm_result, dxl_error = packetHandler2.read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler2.getRxPacketError(dxl_error))

        # Read Dynamixel#3 present position
        dxl3_present_position, dxl_comm_result, dxl_error = packetHandler3.read4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler3.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler3.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl1_goal_position[index], dxl1_present_position, DXL2_ID, dxl2_goal_position[index], dxl2_present_position,DXL3_ID, dxl3_goal_position[index], dxl3_present_position))

        if not ((abs(dxl1_goal_position[index] - dxl1_present_position) > DXL1_MOVING_STATUS_THRESHOLD) and (abs(dxl2_goal_position[index] - dxl2_present_position) > DXL2_MOVING_STATUS_THRESHOLD) and (abs(dxl3_goal_position[index] - dxl3_present_position) > DXL3_MOVING_STATUS_THRESHOLD)):
            break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0    


# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler1.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler1.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler2.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler2.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler2.getRxPacketError(dxl_error))

# Disable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler3.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler3.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler3.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()