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
    if b == 0.0:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.pi/2
    else:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.arctan(a / b)
    return x

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
    servo0 = trig_solve(A,B,C)
    servo1 = trig_solve(D,E,F)
    servo2 = trig_solve(G,H,I)

    #convert servo angles to bits
    servo0 = 2048 + 1024 * (servo0 * (2/np.pi))
    servo1 = 2048 + 1024 * (servo1 * (2/np.pi))
    servo2 = 2048 - 1024 * (servo2 * (2/np.pi))

    x_parasitic = ParasiticMotion(r, theta, phi)
  
    return servo0, servo1, servo2, x_parasitic


# def forwardPosKinematics(servo0,servo1,servo2):
#     L = geom.L         #distal link length in m
#     l = geom.l         #proximal link length in m
#     r = geom.r         #end-effector platform radius in m
#     b = geom.b         #base radius in m

#     #servo angles from bits to radians
#     servo0 = (servo0 - 2048) / (1024 * 2/np.pi)
#     servo1 = (servo1 - 2048) / (1024 * 2/np.pi)
#     servo2 = (servo2 - 2048) / (-1024 * 2/np.pi)

#     B1 = np.asarray([b + l * np.cos(servo0), 0, l * np.sin(servo0)])
#     B2 = np.asarray([0, b + l * np.cos(servo1), l * np.sin(servo1)])
#     B3 = np.asarray([-b - l * np.cos(servo2), 0, l * np.sin(servo2)])

#     A1 = np.asarray([r * np.cos(theta) + x, 0, z - r * np.sin(theta)])
#     A2 = np.asarray([0, r * np.cos(phi), z + r * np.sin(phi)])
#     A3 = np.asarray([-r * np.cos(theta) - x, 0, z + r * np.sin(theta)])

#     x = r * (np.sin(theta) * np.sin(phi))

#     L = np.linalg.norm(B1 - A1)
#     L = np.linalg.norm(B2 - A2)
#     L = np.linalg.norm(B3 - A3)

#     return z, x

def ParasiticMotion(r, theta, phi):
    #Calculate parasitic motion in x direction
    x = r * (np.sin(theta) * np.sin(phi))
    return x

def Force2Current(T_theta, T_phi, F_z,alpha,beta,gamma,Theta_1,Theta_2,Theta_3,z,theta,phi):
    #calculate servo torques
    T = np.empty([3,1])
    l = geom.l
    r = geom.r

    T[0] = -l*(F_z*r*np.sin(beta)*np.sin(gamma + theta)*np.cos(phi) + F_z*r*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)*np.cos(theta) 
        + T_phi*np.sin(beta)*np.sin(gamma)*np.sin(phi)*np.sin(theta) + T_phi*np.sin(beta)*np.sin(gamma + theta) + T_theta*np.sin(beta)*np.sin(gamma)*np.cos(phi) 
        + T_theta*np.sin(gamma)*np.sin(phi)*np.cos(beta)*np.cos(theta))/(r*(np.sin(alpha)*np.sin(beta)*np.sin(gamma + theta)*np.cos(phi) 
        + np.sin(alpha)*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)*np.cos(theta) + np.sin(beta)*np.sin(gamma)*np.sin(alpha - theta)*np.cos(phi) 
        + np.sin(gamma)*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)*np.cos(theta))*np.sin(Theta_1 - alpha))
    T[1] = T_phi*l/(r*(np.sin(beta)*np.cos(phi) + np.sin(phi)*np.cos(beta)*np.cos(theta))*np.sin(Theta_2 - beta))
    T[2] = l*(-F_z*r*np.sin(beta)*np.sin(alpha - theta)*np.cos(phi) - F_z*r*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)*np.cos(theta) 
        + T_phi*np.sin(alpha)*np.sin(beta)*np.sin(phi)*np.sin(theta) - T_phi*np.sin(beta)*np.sin(alpha - theta) + T_theta*np.sin(alpha)*np.sin(beta)*np.cos(phi) 
        + T_theta*np.sin(alpha)*np.sin(phi)*np.cos(beta)*np.cos(theta))/(r*(np.sin(alpha)*np.sin(beta)*np.sin(gamma + theta)*np.cos(phi) 
        + np.sin(alpha)*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)*np.cos(theta) + np.sin(beta)*np.sin(gamma)*np.sin(alpha - theta)*np.cos(phi) 
        + np.sin(gamma)*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)*np.cos(theta))*np.sin(Theta_3 - gamma))
    #calculate servo current limit to achieve desired torque (taken from datasheet graph)
    I = (-0.66733 * T - 0.05492) * 2.69

    return I

def delta_callback_tip(tip_pos_sub, tip_force_sub):
    global servo_angle_pub
    global servo_current_pub
    servo_angle = servo_angles()
    servo_current = servo_angles()
    

    z = tip_pos_sub.point.z
    theta = tip_pos_sub.point.x
    phi = tip_pos_sub.point.y

    servo0, servo1, servo2, x_parasitic  = invPosKinematics(z,theta,phi) # calculate servo angles

    #put servo angles in message format
    servo_angle.header.stamp = rospy.Time.now()
    servo_angle.header.frame_id = "/fcu"
    servo_angle.theta1 = int(servo0)
    servo_angle.theta2 = int(servo1)
    servo_angle.theta3 = int(servo2)
    servo_angle_pub.publish(servo_angle)

    servo0 = servo_angle_sub.theta1
    servo1 = servo_angle_sub.theta2
    servo2 = servo_angle_sub.theta3
    
    T_theta = tip_force_sub.point.x
    T_phi = tip_force_sub.point.y
    F_z = tip_force_sub.point.z

    z, theta, phi = forwardPosKinematics(servo0,servo1,servo2)

    I = Force2Current(T_theta, T_phi, F_z, servo0, servo1, servo2, z, theta, phi) # calculate current limit
    
    servo_current.header.stamp = rospy.Time.now()
    servo_current.header.frame_id = "/fcu"
    servo_current.theta1 = int(I.item(0))
    servo_current.theta2 = int(I.item(1))
    servo_current.theta3 = int(I.item(2))
    servo_current_pub.publish(servo_current)

    parasitic_motion_pub.publish(x_parasitic)
    

if __name__ == '__main__':

    rospy.init_node('delta_main_node', anonymous=True)
    srv = Server(GeomConfig, config_callback)

    robot_name = rospy.get_param('/namespace') 

    # PUBLISHER
    servo_angle_pub = rospy.Publisher(robot_name+'/servo_angles_commanded', servo_angles, queue_size=1) # servo angle publisher
    parasitic_motion_pub = rospy.Publisher(robot_name+'/parasitic_x', Float64, queue_size=1) # parasitic motion publisher
    servo_current_pub = rospy.Publisher(robot_name+'/servo_current_lims', servo_angles, queue_size=1) # servo current publisher

    #SUBSCRIBER
    tip_pos_sub = message_filters.Subscriber(robot_name+'/tip_position', PointStamped) #target angle subscriber
    tip_force_sub = message_filters.Subscriber(robot_name+'/tip_force', Float64) #target force subscriber
    servo_angle_sub = message_filters.Subscriber(robot_name+'/servo_angles_measured', servo_angles) #target angle subscriber

    #TIME SYNC
    ts = message_filters.ApproximateTimeSynchronizer([tip_pos_sub, tip_force_sub], 1, 100)
    
    #CALLBACK
    ts.registerCallback(delta_callback_tip)

    rospy.spin()