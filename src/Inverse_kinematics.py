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
    # print("a= ",a)
    # print("b= ",b)
    # print("c= ",c)
    # print("1st bit: ", c / (np.sqrt(a**2 + b**2)))
    # print("2nd bit: ", a/b)
    #solve the equation: a*sin(x) + b*cos(x) = c
    if b == 0.0:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.pi/2
    #elif c / (np.sqrt(a**2 + b**2)) < 0:
    #    x = np.pi/2 - np.arccos(abs(c / (np.sqrt(a**2 + b**2)))) + np.arctan(a / b)
    else:
        x = np.arccos(c / (np.sqrt(a**2 + b**2))) + np.arctan(a / b)
    # if (x <= np.pi/2) and (x >= -np.pi/2):
    #     print("good solve") 
    # else:
    #     print("SOLVE FAIL")
    #     #quit()

    # print("3rd bit: ", np.arccos(abs(c / (np.sqrt(a**2 + b**2)))))
    return x

#def radians2bits(t):
    #servo angles in dynamixel units
    
   # return t_int

def invPosKinematics(z,theta,phi):
    L = geom.L         #distal link length in m
    l = geom.l         #proximal link length in m
    r = geom.r         #end-effector platform radius in m
    b = geom.b         #base radius in m

    # print("L= ",L)
    # print("l= ",l)
    # print("r= ",r)
    # print("b= ",b)

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

    x_parasitic = ParasiticMotion(r, theta, phi)
  
    return servo0, servo1, servo2, x_parasitic

def ParasiticMotion(r, theta, phi):
    #Calculate parasitic motion in x direction
    x = r * (np.sin(theta) * np.sin(phi))
    return x

def Force2Current(R,servo,z,theta,phi):
    #calculate servo torques
    T = np.empty([3,1])
    l = geom.l

    T[1] = (R * l * np.cos(theta) * np.sin(phi)) / np.sin(servo[1])

    T[2] = (R * l * np.cos(theta) * np.cos(phi) * np.sin(servo[0]) - 
        T[1] * np.cos(servo[1]) * np.sin(servo[0]) + R * l * np.cos(phi) * 
        np.sin(theta) * np.cos(servo[0])) / (np.cos(servo[2]) * 
        np.sin(servo[0]) + np.sin(servo[2]) * np.cos(servo[0]))
    
    T[0] = (R * l * np.cos(theta) * np.cos(phi) - T[1] * np.cos(servo[1]) - 
        T[2] * np.cos(servo[2])) / np.cos(servo[0])

    #calculate servo current limit to achieve desired torque (taken from datasheet graph)
    I = (-0.66733 * T - 0.05492) * 2.69

    return I

def delta_callback_tip(tip_pos_sub, tip_force_sub):
    global servo_angle_pub
    global servo_current_pub
    servo_angle = servo_angles()
    # servo_current = servo_angles()

    z = tip_pos_sub.point.z
    theta = tip_pos_sub.point.x
    phi = tip_pos_sub.point.y

    R = tip_force_sub.data
    
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
        I = int((0.66733 * abs(T) + 0.05492) *1000 / 2.69) - 20 #-20 is a fudge so current can go to zero
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