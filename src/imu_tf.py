#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  5 11:43:00 2020

@author: apola
"""
import rospy

import tf
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

def imu_Cb(sensor_data):
    
    global last_gyro
    global last_time
    global theta
    try:
        last_time
        last_gyro
        theta
    except:
        last_time = rospy.Time.now()
        last_gyro = sensor_data.angular_velocity.y
        theta = 0
        return
    
    if(abs(sensor_data.angular_velocity.y)<0.05):
        current_angular_y = 0
    else:
        current_angular_y = -1.0 * sensor_data.angular_velocity.y
        
    dt = rospy.Time.now().to_time()-last_time.to_time()
    theta += (current_angular_y + last_gyro)*dt/2.
    
    br.sendTransform((16.5, 42.6, 1.0),
                     quaternion_from_euler(0, 0, theta),
                     rospy.Time.now(),
                     "base_link",
                     "map")
    
    last_gyro = current_angular_y
    last_time = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('imu_tf_broadcaster')
    br = tf.TransformBroadcaster()
    sensing_sub = rospy.Subscriber('/camera/imu', Imu, imu_Cb)
    rospy.spin()