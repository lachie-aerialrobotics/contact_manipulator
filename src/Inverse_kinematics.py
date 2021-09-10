#!/usr/bin/env python
import rospy
import numpy as np
import message_filters

from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
from contact_manipulator.msg import servo_angles

from dynamic_reconfigure.server import Server
from contact_manipulator.cfg import GeomConfig

class geom:
    def __init__(self, r, b, l, L):
        self.r = r
        self.b = b
        self.l = l
        self.L = L

def config_callback(config, level): 
    geom.l = config.l
    geom.L = config.L
    geom.r = config.r
    geom.b = config.b
    return config

def trig_solve(a,b,c):
    #solve the equation: a*sin(x) + b*cos(x) = c
    if b == 0.0:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.pi/2
    else:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.arctan(a / b)
    if (x <= np.pi/2) and (x >= -np.pi/2):
        print("good solve") 
    else:
        print("SOLVE FAIL")
        quit()
    return x

#def radians2bits(t):
    #servo angles in dynamixel units
    
   # return t_int

def invPosKinematics(z,theta,phi):
    L = geom.L         #distal link length in m
    l = geom.l         #proximal link length in m
    r = geom.r         #end-effector platform radius in m
    b = geom.b         #base radius in m

    #Solve position kinematics
    A = -z + r * np.sin(theta)
    B = -r * (np.cos(theta) - np.sin(phi) * np.sin(theta)) + b
    C = (L**2 - A**2 - B**2 - l**2) / (2*l)

    D = -z - r * np.cos(theta) * np.sin(phi)
    E = -r * np.cos(phi) + b
    F = (L**2 - D**2 - E**2 - l**2) / (2*l)

    G = -z - r * np.sin(theta)
    H = -r * (np.cos(theta) + np.sin(theta) * np.sin(phi)) + b
    I = (L**2 - G**2 - H**2 - l**2) / (2*l)

    #servo angles in radians
    servo = np.empty([3,1])
    servo[0] = trig_solve(A,B,C)
    servo[1] = trig_solve(D,E,F)
    servo[2] = trig_solve(G,H,I)
  
    return servo

def Force2Current(R,servo,z,theta,phi):
    #calculate servo torques
    T = np.empty([3,1])
    l = geom.l

    T[1] = (R * l * np.cos(theta) * np.sin(phi)) / np.sin(servo[1])

    T[2] = (R * l * np.cos(theta) * np.cos(phi) * np.sin(servo[0]) - 
        T[1] * np.cos(servo[1]) * np.sin(servo[0]) + R * l * np.cos(phi) * 
        np.sin(theta) * np.cos(servo[0])) / (np.cos(servo[2]) * 
        np.sin(servo[0]) + np.sin(servo[2]) * np.cos(servo[0]))
    
    T[0] = (R * geom.l * np.cos(theta) * np.cos(phi) - T[1] * np.cos(servo[1]) - 
        T[2] * np.cos(servo[2])) / np.cos(servo[0])

    #calculate servo current limit to achieve desired torque (taken from datasheet graph)
    I = -1.03925 * T + 0.08021
    return I

def delta_callback_tip(tip_pos_sub, tip_force_sub):
    global servo_angle_pub
    global servo_current_pub
    servo_angle = servo_angles()
    servo_current = servo_angles()

    z = tip_pos_sub.point.z
    theta = tip_pos_sub.point.x
    phi = tip_pos_sub.point.y

    R = tip_force_sub.data
    
    servo  = invPosKinematics(z,theta,phi) # calculate servo angles
    I = Force2Current(R,servo,z,theta,phi) # calculate current limit
    
    #xonvert servo angles to bits
    servo[0] = 2048 + 1024 * (servo[0] * (2/np.pi))
    servo[1] = 2048 - 1024 * (servo[1] * (2/np.pi))
    servo[2] = 2048 + 1024 * (servo[2] * (2/np.pi))

    #put servo angles in message format
    servo_angle.header.stamp = rospy.Time.now()
    servo_angle.header.frame_id = "/fcu"
    servo_angle.theta1 = int(servo.item(0))
    servo_angle.theta2 = int(servo.item(1))
    servo_angle.theta3 = int(servo.item(2))
    servo_angle_pub.publish(servo_angle)

    servo_current.header.stamp = rospy.Time.now()
    servo_current.header.frame_id = "/fcu"
    servo_current.theta1 = int(I.item(0))
    servo_current.theta2 = int(I.item(1))
    servo_current.theta3 = int(I.item(2))
    servo_current_pub.publish(servo_current)

if __name__ == '__main__':
    global servo_vel_pub
    global servo_current_pub

    rospy.init_node('delta_main_node', anonymous=True)
    srv = Server(GeomConfig, config_callback)

    robot_name = rospy.get_param('/namespace') 

    # PUBLISHER
    servo_angle_pub = rospy.Publisher(robot_name+'/servo_angles', servo_angles, queue_size=1) # servo angle publisher
    servo_current_pub = rospy.Publisher(robot_name+'/servo_current_lims', servo_angles, queue_size=1) # servo current publisher

    #SUBSCRIBER
    tip_pos_sub = message_filters.Subscriber(robot_name+'/tip_position', PointStamped) #target angle subscriber
    tip_force_sub = message_filters.Subscriber(robot_name+'/tip_force', Float64) #target force subscriber

    #TIME SYNC
    ts = message_filters.ApproximateTimeSynchronizer([tip_pos_sub, tip_force_sub], 1, 100, allow_headerless=True)
    
    #CALLBACK
    ts.registerCallback(delta_callback_tip)

    rospy.spin()