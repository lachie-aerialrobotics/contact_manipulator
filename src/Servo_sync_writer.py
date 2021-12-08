#!/usr/bin/env python

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

import message_filters
import rospy
import time
from contact_manipulator.msg import servo_angles
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def Initialise():
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupBulkWrite instance
    groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

    # Initialize GroupBulkRead instace for Present Position
    groupBulkRead = GroupBulkRead(portHandler, packetHandler)

    # Open port
    if portHandler.openPort():
        rospy.loginfo("Succeeded to open the port")
    else:
        rospy.loginfo("Failed to open the port")


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        rospy.loginfo("Succeeded to change the baudrate")
    else:
        rospy.loginfo("Failed to change the baudrate")

    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel#%d has been successfully connected" % DXL1_ID)

    # Enable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel#%d has been successfully connected" % DXL2_ID)

    # Enable Dynamixel#3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        rospy.loginfo("Dynamixel#%d has been successfully connected" % DXL3_ID)

    # Add parameter storage for Dynamixel#1 present position
    dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkRead addparam failed" % DXL1_ID)

    # Add parameter storage for Dynamixel#2 present position
    dxl_addparam_result = groupBulkRead.addParam(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkRead addparam failed" % DXL2_ID)

    # Add parameter storage for Dynamixel#3 present position
    dxl_addparam_result = groupBulkRead.addParam(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkRead addparam failed" % DXL3_ID)
    return groupBulkWrite, groupBulkRead, portHandler, packetHandler

def ServoCallback(servo_angle_sub, servo_current_sub): #servo_current_sub):
    tic1 = time.clock()
    dxl_goal_position_1 = servo_angle_sub.theta1
    dxl_goal_position_2 = servo_angle_sub.theta2
    dxl_goal_position_3 = servo_angle_sub.theta3

    dxl_goal_current_1 = servo_current_sub.theta1
    dxl_goal_current_2 = servo_current_sub.theta2
    dxl_goal_current_3 = servo_current_sub.theta3
    
    tic5 = time.clock()
    # Allocate goal position value into byte array
    param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_1)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_1)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_1)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_1))]
    param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_2)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_2)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_2)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_2))]
    param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_3)), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_3)), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_3)), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_3))]

    param_goal_current_1 = [DXL_LOBYTE(dxl_goal_current_1), DXL_HIBYTE(dxl_goal_current_1)] 
    param_goal_current_2 = [DXL_LOBYTE(dxl_goal_current_2), DXL_HIBYTE(dxl_goal_current_2)] 
    param_goal_current_3 = [DXL_LOBYTE(dxl_goal_current_3), DXL_HIBYTE(dxl_goal_current_3)] 
    toc5 = time.clock()

    tic2 = time.clock()
    # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position_1)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkWrite addparam position failed" % DXL1_ID)

    # Add Dynamixel#2 goal position value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position_2)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkWrite addparam position failed" % DXL2_ID)

    # Add Dynamixel#3 goal position value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL3_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position_3)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkWrite addparam position failed" % DXL3_ID)
    
    # Bulkwrite goal position and LED value
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()
    toc2 = time.clock()
 
    tic3 = time.clock()
    # Add Dynamixel#1 goal current value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_PRO_GOAL_CURRENT, LEN_PRO_GOAL_CURRENT, param_goal_current_1)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkWrite addparam current failed" % DXL1_ID)

    # Add Dynamixel#2 goal current value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL2_ID, ADDR_PRO_GOAL_CURRENT, LEN_PRO_GOAL_CURRENT, param_goal_current_2)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkWrite addparam current failed" % DXL2_ID)

    # Add Dynamixel#3 goal current value to the Bulkwrite parameter storage
    dxl_addparam_result = groupBulkWrite.addParam(DXL3_ID, ADDR_PRO_GOAL_CURRENT, LEN_PRO_GOAL_CURRENT, param_goal_current_3)
    if dxl_addparam_result != True:
        rospy.loginfo("[ID:%03d] groupBulkWrite addparam current failed" % DXL3_ID)

    # Bulkwrite goal position and LED value
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()
    toc3 = time.clock()

    tic4 = time.clock()
    # Bulkread present position and LED status
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkRead.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_getdata_result != True:
        rospy.loginfo("[ID:%03d] groupBulkRead getdata failed" % DXL1_ID)

    # Check if groupbulkread data of Dynamixel#2 is available
    dxl_getdata_result = groupBulkRead.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_getdata_result != True:
        rospy.loginfo("[ID:%03d] groupBulkRead getdata failed" % DXL2_ID)

    # Check if groupbulkread data of Dynamixel#3 is available
    dxl_getdata_result = groupBulkRead.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    if dxl_getdata_result != True:
        rospy.loginfo("[ID:%03d] groupBulkRead getdata failed" % DXL3_ID)

    # Get present position value
    dxl_present_position_1 = groupBulkRead.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    dxl_present_position_2 = groupBulkRead.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    dxl_present_position_3 = groupBulkRead.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
    toc4 = time.clock()

    theta = servo_angles_write(dxl_present_position_1, dxl_present_position_2, dxl_present_position_3)
    servo_angle_pub.publish(theta)

    toc1 = time.clock()
    timer1 = toc1 - tic1
    timer2 = toc2 - tic2
    timer3 = toc3 - tic3
    timer4 = toc4 - tic4
    timer5 = toc5 - tic5
    rospy.loginfo("LOOP_TIME:                 ")
    rospy.loginfo(timer1)
    rospy.loginfo("GROUP_BULK_WRITE_POS_TIME: ")
    rospy.loginfo(timer2)
    rospy.loginfo("GROUP_BULK_WRITE_CUR_TIME: ")
    rospy.loginfo(timer3)
    rospy.loginfo("GROUP_BULK_READ_POS_TIME:  ")
    rospy.loginfo(timer4)
    rospy.loginfo("BYTE_ARRAY_ALLOCATION_TIME:")
    rospy.loginfo(timer5)

def servo_angles_write(theta_1, theta_2, theta_3):
    theta = servo_angles()
    theta.header.stamp = rospy.Time.now()
    theta.header.frame_id = "/fcu"
    theta.theta1 = theta_1
    theta.theta2 = theta_2
    theta.theta3 = theta_3
    return theta

    
if __name__ == '__main__':
    # Control table address
    ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
    ADDR_PRO_LED_RED            = 65
    ADDR_PRO_GOAL_POSITION      = 116
    ADDR_PRO_PRESENT_POSITION   = 132
    ADDR_PRO_GOAL_CURRENT       = 102

    # Data Byte Length
    LEN_PRO_LED_RED             = 1
    LEN_PRO_GOAL_POSITION       = 4
    LEN_PRO_PRESENT_POSITION    = 4
    LEN_PRO_GOAL_CURRENT        = 2

    # Protocol version
    PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
    DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
    DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
    BAUDRATE                    = 3000000             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque


    rospy.init_node('Servo_writer', anonymous=True)
    robot_name = rospy.get_param('/namespace')
    servo_angle_sub = message_filters.Subscriber(robot_name+'/servo_angles_setpoint', servo_angles) #target angle subscriber
    servo_current_sub = message_filters.Subscriber(robot_name+'/servo_current_lims', servo_angles) #current limit subscriber
    servo_angle_pub = rospy.Publisher(robot_name+'/servo_angles', servo_angles, queue_size=1) # servo angle publisher
    groupBulkWrite, groupBulkRead, portHandler, packetHandler = Initialise()
    ts = message_filters.ApproximateTimeSynchronizer([servo_angle_sub, servo_current_sub], 1, 10)
    ts.registerCallback(ServoCallback)
    rospy.spin()

    # Clear bulkread parameter storage
    groupBulkRead.clearParam()

    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()