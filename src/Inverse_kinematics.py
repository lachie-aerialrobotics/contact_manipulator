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
    @staticmethod
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
        self.T1 = force.point.x
        self.T2 = force.point.y
        self.T3 = force.point.z

    def callback_ang(self):  #return servo angle message         
        self.theta1, self.theta2, self.theta3, issolved = self.inverse_kinematics(self.theta, self.phi, self.z, geom.L, geom.l, geom.r, geom.b)
        
        if issolved == True:
            self.thetb1 = self.rads2bits(self.theta1,"pos")
            self.thetb2 = self.rads2bits(self.theta2,"pos")
            self.thetb3 = self.rads2bits(self.theta3,"neg")
            cache.thetb1 = self.thetb1
            cache.thetb2 = self.thetb2
            cache.thetb3 = self.thetb3
        else:
            self.thetb1 = cache.thetb1
            self.thetb2 = cache.thetb2
            self.thetb3 = cache.thetb3

        ang_msg = ServoMsg(self.thetb1,self.thetb2,self.thetb3).msg
        return ang_msg

    def callback_crrnt(self): #return servo current message

        I1 = self.torque2current(self.T1)
        I2 = self.torque2current(self.T2)
        I3 = self.torque2current(self.T3)

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
        theta1, issolved1 = self.trig_solve(A,B,C)
        theta2, issolved2 = self.trig_solve(D,E,F)
        theta3, issolved3 = self.trig_solve(G,H,I)

        #check all 3 solutions came out valid
        issolved = (issolved1 & issolved2) & issolved3

        return theta1, theta2, theta3, issolved

    def solve_checker(self,t):
        #check that input to arccos is valid 
        # (this is the bit that goes wrong when manipulator workspace is exceeded)
        invalid = np.isnan(t) or (t >= 1) or (t <= -1)
        if invalid == True:
            rospy.logwarn("Workspace of manipulator exceeded!")
            issolved = False
        else:
            issolved = True
        return issolved
    
    def trig_solve(self,a,b,c):
        # solve equations using tan substitution
        A = c / (np.sqrt(a**2 + b**2))
        issolved = self.solve_checker(A)
        if issolved == True:
            if b == 0.0:
                x = np.arccos(A) + np.pi/2
            else:
                x = np.arccos(A) + np.arctan(a / b) 
        else:
            x = np.nan
        return x, issolved
    
    def rads2bits(self,theta,dir):
        #convert from radians to bit values recongnised by dynamixels
        if dir == "pos":
            thetb = int(2048 + 1024 * (theta * (2/np.pi)))
        elif dir == "neg":
            thetb = int(2048 - 1024 * (theta * (2/np.pi)))
        return thetb 

    def bits2rads(self,thetb,dir):
        #convert from bits back to radians
        if dir =="pos":
            theta = float((thetb - 2048)/1024 * np.pi/2)
        elif dir == "neg":
            theta = float((thetb - 2048)/-1024 * np.pi/2)
        return theta

    def torque2current(self,T):
        #calculate servo current limit to achieve desired torque (taken from datasheet graph)
        I = int((0.66733 * abs(T) + 0.05492) *1000 / 2.69)
        if I > 648:
            I = 648 #do not let current exceed limit
        return I
class cache: #save most recent valid values of servo angles in case inverse kinematics breaks
    def __init__(self, theta1, theta2, theta3):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
class ServoMsg: #class to assign values to servo_angles message format  
    def __init__(self, Theta1, Theta2, Theta3): 
        self.msg = servo_angles()
        self.msg.header.stamp = rospy.Time.now() 
        self.msg.header.frame_id = "/servos"
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

        ts = message_filters.ApproximateTimeSynchronizer([self.sub_pos, self.sub_force], 1, 10)
        ts.registerCallback(self.tip_callback)
        
    def tip_callback(self, sub_pos, sub_force): #callback calculates servo angles/torques
        ang = delta(sub_pos,sub_force).callback_ang()
        self.pub_ang.publish(ang)
        crrnt = delta(sub_pos,sub_force).callback_crrnt()
        self.pub_crrnt.publish(crrnt)
        
if __name__ == '__main__': #initialise node and run loop
    rospy.init_node('delta_main_node', anonymous=True)
    srv = Server(GeomConfig, geom.config_callback)
    Controller()
    rospy.spin()