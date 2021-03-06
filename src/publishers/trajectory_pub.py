#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from dynamic_reconfigure.server import Server
from contact_manipulator.cfg import TrajectoryConfig

class Trajectory:
    ros_rate = 100
    
    def __init__(self):
        robot_name = rospy.get_param('/namespace')
        srv = Server(TrajectoryConfig, config_callback)
        self.pub_pos = rospy.Publisher(robot_name+'/tip_position', PointStamped, queue_size=1)
        self.pub_force = rospy.Publisher(robot_name+'/tip_force', PointStamped, queue_size=1)
        rate = rospy.Rate(self.ros_rate) # in Hz
        
        while not rospy.is_shutdown():
            pos = callback_pos().pos_msg
            force = callback_force().force_msg
            self.pub_pos.publish(pos)
            self.pub_force.publish(force)
            rate.sleep()

class callback_pos:
    pos_msg = PointStamped()
    pos_msg.header.frame_id = "/fcu"
    
    def __init__(self):
        self.pos_msg.header.stamp = rospy.Time.now()
        global t
        t = t + cfg.v / (cfg.r * Trajectory.ros_rate)

        if cfg.mode == 0: #static
            self.pos_msg.point.x = cfg.theta
            self.pos_msg.point.y = cfg.phi
            self.pos_msg.point.z = cfg.z
        if cfg.mode == 1: #line z
            self.pos_msg.point.y = cfg.phi
            self.pos_msg.point.x = cfg.theta
            self.pos_msg.point.z = cfg.z + cfg.r * np.cos(t)
        if cfg.mode == 2: #line phi
            self.pos_msg.point.z = cfg.z
            self.pos_msg.point.x = cfg.theta
            self.pos_msg.point.y = cfg.phi + cfg.r * np.cos(t)
        if cfg.mode == 3: #line theta
            self.pos_msg.point.y = cfg.phi
            self.pos_msg.point.z = cfg.z
            self.pos_msg.point.x = cfg.theta + cfg.r * np.cos(t)
        if cfg.mode == 4: #circle
            self.pos_msg.point.x = cfg.theta + cfg.r * np.cos(t)
            self.pos_msg.point.y = cfg.phi + cfg.r * np.sin(t)
            self.pos_msg.point.z = cfg.z 

class callback_force:
    force_msg=PointStamped()
    force_msg.header.frame_id = "/fcu"

    def __init__(self):
        self.force_msg.header.stamp = rospy.Time.now()
        self.force_msg.point.x = cfg.T_1
        self.force_msg.point.y = cfg.T_2
        self.force_msg.point.z = cfg.T_3

class cfg:
    def __init__(self, r, theta, phi, z, v, T_1, T_2, T_3, mode):
        self.r = r
        self.theta = theta
        self.phi = phi
        self.z = z
        self.v = v
        self.T_1 = T_1
        self.T_2 = T_2
        self.T_3 = T_3
        self.mode = mode

def config_callback(config, level): 
    cfg.r = config.r
    cfg.theta = config.theta
    cfg.phi = config.phi
    cfg.z = config.z
    cfg.v = config.v  
    cfg.T_1 = config.T_1
    cfg.T_2 = config.T_2
    cfg.T_3 = config.T_3
    cfg.mode = config.mode
    return config
         
if __name__ == '__main__':    
    rospy.init_node('talker_target', anonymous=True)
    global t
    t = 0.
    
    try:
        Trajectory()
    except rospy.ROSInterruptException:
        pass