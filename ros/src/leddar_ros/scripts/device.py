#!/usr/bin/env python

'''
Created on Mar 2, 2018

@author: Maxime Lemonnier

@file: device.py

@summary: leddar_ros device node definition

@copyright: Copyright (c) 2018 LeddarTech Inc. All rights reserved.
'''

import leddar
from leddar_utils import clouds
import rospy
import math
import numpy as np
import time

from rospy.numpy_msg import numpy_msg
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from leddar_ros.msg import Specs
import ros_numpy

TIMESTAMPS, DISTANCE, AMPLITUDE = range(3)

if __name__ == '__main__':

    rospy.init_node('leddar_ros', disable_signals=True)

    dev = leddar.Device()

    frame_id = rospy.get_param('~frame_id', 'scan')
    param1 = rospy.get_param('~param1')
    device_type = rospy.get_param('~device_type',"not specified")
    param3 = rospy.get_param('~param3', 0)
    param4 = rospy.get_param('~param4', 0)
    param1 = str(param1) # Be sure its a string (for CANbus)
    param3 = int(param3)
    param4 = int(param4)

    dev_type = 0
    if(device_type != "not specified"):
        dev_type = leddar.device_types[device_type]

    if not dev.connect(param1, dev_type, param3, param4):
        err_msg = 'Error connecting to device type {0} with connection info {1}/{2}/{3}.'.format(device_type, param1, str(param3), str(param4))
        rospy.logerr(err_msg)
        raise RuntimeError(err_msg)

    specs = Specs()
    specs.v, specs.h, = [int(dev.get_property_value(x)) for x in ["ID_VERTICAL_CHANNEL_NBR", "ID_HORIZONTAL_CHANNEL_NBR"]]
    specs.v_fov, specs.h_fov = [float(dev.get_property_value(x)) for x in ["ID_VFOV", "ID_HFOV"]]

    pub_specs = rospy.Publisher('specs', Specs, queue_size=100)
    pub_specs.publish(specs)
    pub_laser = rospy.Publisher('scan_laser', LaserScan, queue_size=100)
    frame_id = rospy.get_param('~frame_id', 'scan')
    
    def echoes_callback(echo):
        echo['data'] = echo['data'][np.bitwise_and(echo['data']['flags'], 0x01).astype(np.bool)] #keep valid echoes only
        indices, flags, distances, amplitudes = [echo['data'][x] for x in ['indices', 'flags', 'distances', 'amplitudes']]
        stamp = rospy.Time.now()

        if pub_laser.get_num_connections() > 0:
            laserscan = LaserScan()

            laserscan.header.stamp = stamp
            laserscan.header.frame_id = frame_id

            laserscan.angle_min = -specs.h_fov/(360) * math.pi
            laserscan.angle_max = -laserscan.angle_min
            laserscan.angle_increment = np.deg2rad(specs.h_fov/specs.h)
           
            laserscan.time_increment = (1/100)/specs.h
           
            laserscan.range_min = .05
            laserscan.range_max = 30

            laserscan.ranges = distances
            laserscan.intensities = amplitudes

            pub_laser.publish(laserscan)


    dev.set_callback_echo(echoes_callback)
    dev.set_data_mask(leddar.data_masks["DM_ECHOES"])
    dev.set_data_thread_delay(1000)
    dev.start_data_thread()
    rospy.spin()

