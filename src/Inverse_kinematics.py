#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from geometry_msgs.msg import PointStamped
from contact_manipulator.msg import servo_angles
from dynamic_reconfigure.server import Server
from contact_manipulator.cfg import GeomConfig
class geom: #data from dynamic reconfigure
    def __init__(self, r, b, l, L):
        self.l = l  #distal link length in m
        self.L = L  #proximal link length in m
        self.r = r  #end-effector platform radius in m
        self.b = b  #base radius in m   
    def config_callback(config, level): 
        geom.l = config.l
        geom.L = config.L
        geom.r = config.r
        geom.b = config.b
        return config
class delta:
    def __init__(self, pos, force):
        #take subscribed messages and store as nice variables
        self.theta = pos.point.x
        self.phi = pos.point.y
        self.z = pos.point.z
        self.T_theta = force.point.x
        self.T_phi = force.point.y
        self.F_z = force.point.z
        
    def callback(self): #callback returns servo angle/torque messages
        ang = self.callback_ang()
        crrnt = self.callback_crrnt()
        return ang, crrnt

    def callback_ang(self):  #return servo angle message   

        self.theta1, self.theta2, self.theta3 = self.inverse_kinematics(self.theta, self.phi, self.z, geom.L, geom.l, geom.r, geom.b)

        thetb1 = self.rads2bits(self.theta1,"pos")
        thetb2 = self.rads2bits(self.theta2,"pos")
        thetb3 = self.rads2bits(self.theta3,"neg")

        ang_msg = ServoMsg(thetb1,thetb2,thetb3).msg

        return ang_msg

    def callback_crrnt(self): #return servo current message

        T1, T2, T3 = self.torque_limits(self.theta, self.phi, self.z, self.T_theta, self.T_phi, self.F_z, geom.L, geom.l, geom.r, geom.b)

        I1 = self.torque2current(T1)
        I2 = self.torque2current(T2)
        I3 = self.torque2current(T3)

        crrnt_msg = ServoMsg(I1,I2,I3).msg

        return crrnt_msg

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
        theta1 = self.trig_solve(A,B,C)
        theta2 = self.trig_solve(D,E,F)
        theta3 = self.trig_solve(G,H,I)

        return theta1, theta2, theta3

    def torque_limits(self, theta, phi, z, T_theta, T_phi, F_z, L, l, r, b):
        #calculate torque limits for each servo
        alpha = np.arcsin((z - r * np.sin(theta) - l * np.sin(self.theta1))/L)
        beta = np.arccos((b + l * np.cos(self.theta2) - r * np.cos(phi))/L)
        gamma = np.arcsin((z + r * np.sin(theta) - l * np.sin(self.theta3))/L)

        # T1 = -l*(F_z*r*np.sin(beta)*np.sin(gamma + theta)*np.cos(phi) 
        #     + F_z*r*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)*np.cos(theta) 
        #     + T_phi*np.sin(beta)*np.sin(gamma)*np.sin(phi)*np.sin(theta) 
        #     + T_phi*np.sin(beta)*np.sin(gamma + theta) + T_theta*np.sin(beta)
        #     *np.sin(gamma)*np.cos(phi) + T_theta*np.sin(gamma)*np.sin(phi)*np.cos(beta)
        #     *np.cos(theta))/(r*(np.sin(alpha)*np.sin(beta)*np.sin(gamma + theta)
        #     *np.cos(phi) + np.sin(alpha)*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)
        #     *np.cos(theta) + np.sin(beta)*np.sin(gamma)*np.sin(alpha - theta)*np.cos(phi) 
        #     + np.sin(gamma)*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)*np.cos(theta))
        #     *np.sin(self.theta1 - alpha))
    
        # T2 = T_phi*l/(r*(np.sin(beta)*np.cos(phi) + np.sin(phi)*np.cos(beta)
        #     *np.cos(theta))*np.sin(self.theta2 - beta))
        
        # T3 = l*(-F_z*r*np.sin(beta)*np.sin(alpha - theta)*np.cos(phi) - F_z*r*np.sin(phi)
        #     *np.sin(alpha - theta)*np.cos(beta)*np.cos(theta) + T_phi*np.sin(alpha)
        #     *np.sin(beta)*np.sin(phi)*np.sin(theta) - T_phi*np.sin(beta)*np.sin(alpha 
        #     - theta) + T_theta*np.sin(alpha)*np.sin(beta)*np.cos(phi) + T_theta*np.sin(alpha)
        #     *np.sin(phi)*np.cos(beta)*np.cos(theta))/(r*(np.sin(alpha)*np.sin(beta)
        #     *np.sin(gamma + theta)*np.cos(phi) + np.sin(alpha)*np.sin(phi)*np.sin(gamma 
        #     + theta)*np.cos(beta)*np.cos(theta) + np.sin(beta)*np.sin(gamma)*np.sin(alpha 
        #     - theta)*np.cos(phi) + np.sin(gamma)*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)
        #     *np.cos(theta))*np.sin(self.theta3 - gamma))

        T1 = -l*(F_z*r*np.sin(beta)*np.sin(gamma + theta)*np.cos(phi) + F_z*r*np.sin(phi)*np.sin(gamma + theta)*np.cos(beta)*np.cos(theta) + T_phi*np.sin(beta)*np.sin(gamma)*np.sin(phi)*np.sin(theta) + T_phi*np.sin(beta)*np.sin(gamma + theta) + T_theta*np.sin(beta)*np.sin(gamma)*np.cos(phi) + T_theta*np.sin(gamma)*np.sin(phi)*np.cos(beta)*np.cos(theta))/(r*(np.sin(alpha)*np.sin(gamma + theta) + np.sin(gamma)*np.sin(alpha - theta))*(np.sin(beta)*np.cos(phi) + np.sin(phi)*np.cos(beta)*np.cos(theta))*np.sin(self.theta1 - alpha))
        T2 = -T_phi*l/(r*(np.sin(beta)*np.cos(phi) + np.sin(phi)*np.cos(beta)*np.cos(theta))*np.sin(self.theta2 - beta))
        T3 = -l*(F_z*r*np.sin(beta)*np.sin(alpha - theta)*np.cos(phi) + F_z*r*np.sin(phi)*np.sin(alpha - theta)*np.cos(beta)*np.cos(theta) - T_phi*np.sin(alpha)*np.sin(beta)*np.sin(phi)*np.sin(theta) + T_phi*np.sin(beta)*np.sin(alpha - theta) - T_theta*np.sin(alpha)*np.sin(beta)*np.cos(phi) - T_theta*np.sin(alpha)*np.sin(phi)*np.cos(beta)*np.cos(theta))/(r*(np.sin(alpha)*np.sin(gamma + theta) + np.sin(gamma)*np.sin(alpha - theta))*(np.sin(beta)*np.cos(phi) + np.sin(phi)*np.cos(beta)*np.cos(theta))*np.sin(self.theta3 - gamma))

        return T1, T2, T3 
    
    def trig_solve(self,a,b,c):
        # solve equations using tan substitution
        if b == 0.0:
            x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.pi/2
        else:
            x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.arctan(a / b)
        return x   
    
    def rads2bits(self,theta,dir):
        #convert from radians to bit values recongnised by dynamixels
        if dir == "pos":
            thetb = int(2048 + 1024 * (theta * (2/np.pi)))
        elif dir == "neg":
            thetb = int(2048 - 1024 * (theta * (2/np.pi)))
        return thetb 

    def torque2current(self,T):
        #calculate servo current limit to achieve desired torque (taken from datasheet graph)
        I = int((0.66733 * abs(T) + 0.05492) *1000 / 2.69)
        if I > 648:
            I = 648 #do not let current exceed limit
        return I
class ServoMsg: #class to assign values to servo_angles message format
    msg = servo_angles()
    msg.header.frame_id = "/servos"
    def __init__(self, Theta1, Theta2, Theta3): 
        self.msg.header.stamp = rospy.Time.now() 
        self.msg.theta1 = Theta1
        self.msg.theta2 = Theta2
        self.msg.theta3 = Theta3
        
class Controller: #init publishers and subscribers
    def __init__(self):
        robot_name = rospy.get_param('/namespace') 
        self.pub_ang = rospy.Publisher(robot_name+'/servo_angles_setpoint', servo_angles, queue_size=1) # servo angle publisher
        self.pub_crrnt = rospy.Publisher(robot_name+'/servo_current_lims', servo_angles, queue_size=1) # servo current publisher

        self.sub_pos = message_filters.Subscriber(robot_name+'/tip_position', PointStamped) #target angle subscriber
        self.sub_force = message_filters.Subscriber(robot_name+'/tip_force', PointStamped) #target force subscriber
        # self.sub_ang = message_filters.Subscriber(robot_name+'/tip_position', PointStamped) #servo angle subscriber

        ts = message_filters.ApproximateTimeSynchronizer([self.sub_pos, self.sub_force], 1, 100)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_pos, sub_force): #callback calculates servo angles/torques
        ang, crrnt = delta(sub_pos,sub_force).callback()
        self.pub_ang.publish(ang)
        self.pub_crrnt.publish(crrnt)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_main_node', anonymous=True)
    srv = Server(GeomConfig, geom.config_callback)
    Controller()
    rospy.spin()