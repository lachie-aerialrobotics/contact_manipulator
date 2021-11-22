#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
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
class callback_force:
    crrnt_msg = servo_angles()
    crrnt_msg.header.frame_id = "/servos"
    def __init__(self, theta, phi, z, T_theta, T_phi, F_z):
        L = geom.L         #distal link length in m
        l = geom.l         #proximal link length in m
        r = geom.r         #end-effector platform radius in m
        b = geom.b         #base radius in m

        alpha = np.arcsin((z - r * np.sin(theta) - l * np.sin(Servo.Theta_1))/L)
        beta = np.arccos((b + l * np.cos(Servo.Theta_2) - r * np.cos(phi))/L)
        gamma = np.arcsin((z + r * np.sin(theta) - l * np.sin(Servo.Theta_3))/L)

        T1 = -l*(F_z*r*np.sin(beta)*np.sin(gamma + theta)*np.cos(phi) 
            + F_z*r*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)*np.cos(theta) 
            + T_phi*np.sin(beta)*np.sin(gamma)*np.sin(phi)*np.sin(theta) 
            + T_phi*np.sin(beta)*np.sin(gamma + theta) + T_theta*np.sin(beta)
            *np.sin(gamma)*np.cos(phi) + T_theta*np.sin(gamma)*np.sin(phi)*np.cos(beta)
            *np.cos(theta))/(r*(np.sin(alpha)*np.sin(beta)*np.sin(gamma + theta)
            *np.cos(phi) + np.sin(alpha)*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)
            *np.cos(theta) + np.sin(beta)*np.sin(gamma)*np.sin(alpha - theta)*np.cos(phi) 
            + np.sin(gamma)*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)*np.cos(theta))
            *np.sin(Servo.Theta_1 - alpha))
        
        T2 = T_phi*l/(r*(np.sin(beta)*np.cos(phi) + np.sin(phi)*np.cos(beta)
            *np.cos(theta))*np.sin(Servo.Theta_2 - beta))
        
        T3 = l*(-F_z*r*np.sin(beta)*np.sin(alpha - theta)*np.cos(phi) - F_z*r*np.sin(phi)
            *np.sin(alpha - theta)*np.cos(beta)*np.cos(theta) + T_phi*np.sin(alpha)
            *np.sin(beta)*np.sin(phi)*np.sin(theta) - T_phi*np.sin(beta)*np.sin(alpha 
            - theta) + T_theta*np.sin(alpha)*np.sin(beta)*np.cos(phi) + T_theta*np.sin(alpha)
            *np.sin(phi)*np.cos(beta)*np.cos(theta))/(r*(np.sin(alpha)*np.sin(beta)
            *np.sin(gamma + theta)*np.cos(phi) + np.sin(alpha)*np.sin(phi)*np.sin(gamma 
            + theta)*np.cos(beta)*np.cos(theta) + np.sin(beta)*np.sin(gamma)*np.sin(alpha 
            - theta)*np.cos(phi) + np.sin(gamma)*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)
            *np.cos(theta))*np.sin(Servo.Theta_3 - gamma))
    
        #calculate servo current limit to achieve desired torque (taken from datasheet graph)
        I1 = abs(int((-0.66733 * T1 - 0.05492) *1000 / 2.69))
        I2 = abs(int((-0.66733 * T2 - 0.05492) *1000 / 2.69))
        I3 = abs(int((-0.66733 * T3 - 0.05492) *1000 / 2.69))

        self.crrnt_msg.theta1 = I1
        self.crrnt_msg.theta2 = I2
        self.crrnt_msg.theta3 = I3
        self.crrnt_msg.header.stamp = rospy.Time.now()

class Servo:
    def __init__(self, Theta_1, Theta_2, Theta_3):
        self.Theta_1 = Theta_1
        self.Theta_2 = Theta_2
        self.Theta_3 = Theta_3
class callback_ang:
    ang_msg = servo_angles()
    ang_msg.header.frame_id = "/servos"
    def __init__(self, theta, phi, z):
        L = geom.L         #distal link length in m
        l = geom.l         #proximal link length in m
        r = geom.r         #end-effector platform radius in m
        b = geom.b         #base radius in m

        Servo.Theta_1, Servo.Theta_2, Servo.Theta_3 = self.inverse_kinematics(theta, phi, z, L, l, r, b)
        
        #convert servo angles to bits
        self.ang_msg.theta1 = int(2048 + 1024 * (Servo.Theta_1 * (2/np.pi)))
        self.ang_msg.theta2 = int(2048 + 1024 * (Servo.Theta_2 * (2/np.pi)))
        self.ang_msg.theta3 = int(2048 - 1024 * (Servo.Theta_3 * (2/np.pi)))
        self.ang_msg.header.stamp = rospy.Time.now()

    def inverse_kinematics(self, theta, phi, z, L, l, r, b):
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
        servo1 = self.trig_solve(A,B,C)
        servo2 = self.trig_solve(D,E,F)
        servo3 = self.trig_solve(G,H,I)

        return servo1, servo2, servo3
    
    def trig_solve(self,a,b,c):
        if b == 0.0:
            x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.pi/2
        else:
            x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.arctan(a / b)
        return x    
class Controller:
    def __init__(self):
        robot_name = rospy.get_param('/namespace') 
        self.pub_ang = rospy.Publisher(robot_name+'/servo_angles_setpoint', servo_angles, queue_size=1) # servo angle publisher
        self.pub_crrnt = rospy.Publisher(robot_name+'/servo_current_lims', servo_angles, queue_size=1) # servo current publisher

        self.sub_pos = message_filters.Subscriber(robot_name+'/tip_position', PointStamped) #target angle subscriber
        self.sub_force = message_filters.Subscriber(robot_name+'/tip_force', PointStamped) #target force subscriber
        
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_pos, self.sub_force], 1, 100)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_pos, sub_force):
        theta = sub_pos.point.x
        phi = sub_pos.point.y
        z = sub_pos.point.z
        T_theta = sub_force.point.x
        T_phi = sub_force.point.y
        F_z = sub_force.point.z

        ang = callback_ang(theta, phi, z).ang_msg
        self.pub_ang.publish(ang)
        crrnt = callback_force(theta, phi, z, T_theta, T_phi, F_z).crrnt_msg
        self.pub_crrnt.publish(crrnt)
        

if __name__ == '__main__':
    rospy.init_node('delta_main_node', anonymous=True)
    srv = Server(GeomConfig, config_callback)
    Controller()
    rospy.spin()