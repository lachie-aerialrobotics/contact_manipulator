#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped

from dynamic_reconfigure.server import Server
from contact_manipulator.cfg import TrajectoryConfig

class trajectory:
    def __init__(self, r, theta, phi, z, v, R, mode):
        self.r = r
        self.theta = theta
        self.phi = phi
        self.z = z
        self.v = v
        self.R = R
        self.mode = mode

def config_callback(config, level): 
    trajectory.r = config.r
    trajectory.theta = config.theta
    trajectory.phi = config.phi
    trajectory.z = config.z
    trajectory.v = config.v  
    trajectory.R = config.R
    trajectory.mode = config.mode
    return config

def talker_target():
    global ros_rate
    robot_name = rospy.get_param('/namespace')
    srv = Server(TrajectoryConfig, config_callback)
    
    pub_target_pos = rospy.Publisher(robot_name+'/tip_position', PointStamped, queue_size=1)
    pub_target_force = rospy.Publisher(robot_name+'/tip_force', Float64, queue_size=1)
    
    target_pos = PointStamped()
    target_pos.header.frame_id = "/fcu"

    target_force = Float64()
    #target_force.header.frame_id = "/fcu"

    theta_1 = 0

    while not rospy.is_shutdown():
        if trajectory.mode == 4: #draw a circle!
            theta_1 = theta_1 + trajectory.v / (trajectory.r * ros_rate)
        
            target_pos.header.stamp = rospy.Time.now()
            target_pos.point.x = trajectory.theta + trajectory.r * np.cos(theta_1)
            target_pos.point.y = trajectory.phi + trajectory.r * np.sin(theta_1)
            target_pos.point.z = trajectory.z

            #theta_dot_1 = trajectory.v / (trajectory.r/2)
        
        elif trajectory.mode == 1: #draw a line in z
            target_pos.header.stamp = rospy.Time.now()
            target_pos.point.y = trajectory.phi
            target_pos.point.x = trajectory.theta

            theta_1 = theta_1 + trajectory.v / (trajectory.r * ros_rate)
            target_pos.point.z = trajectory.z + trajectory.r * np.cos(theta_1)

        elif trajectory.mode == 2: #draw a line in phi
            target_pos.header.stamp = rospy.Time.now()
            target_pos.point.z = trajectory.z
            target_pos.point.x = trajectory.theta

            theta_1 = theta_1 + trajectory.v / (trajectory.r * ros_rate)
            target_pos.point.y = trajectory.phi + trajectory.r * np.cos(theta_1)

        elif trajectory.mode == 3: #draw a line in theta
            target_pos.header.stamp = rospy.Time.now()
            target_pos.point.y = trajectory.phi
            target_pos.point.z = trajectory.z

            theta_1 = theta_1 + trajectory.v / (trajectory.r * ros_rate)
            target_pos.point.x = trajectory.theta + trajectory.r * np.cos(theta_1)


        elif trajectory.mode == 0: #static!
            target_pos.header.stamp = rospy.Time.now()
            
            target_pos.point.x = trajectory.theta
            target_pos.point.y = trajectory.phi
            target_pos.point.z = trajectory.z
               
        pub_target_pos.publish(target_pos)

        target_force.data = trajectory.R
        #target_force.header.stamp = rospy.Time.now()
        pub_target_force.publish(target_force)
        
        rate.sleep()
            
if __name__ == '__main__':    
    rospy.init_node('talker_target', anonymous=True)
    global ros_rate
    ros_rate = 100
    rate = rospy.Rate(ros_rate) # in Hz
    try:
        talker_target()
    except rospy.ROSInterruptException:
        pass
