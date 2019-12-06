#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 3 19:22:40 2019

@author: ag345
"""

import rospy
import numpy as np
from std_msgs.msg import String
from pedestrain_tracking.msg import Floatlist
from centroidtracker import CentroidTracker
#from centroidtracker import CentroidTracker

#firstly subscribe to the centroids topic, because the msg type is std_msgs/String, 
# we need to convert it to dictionary

def callback(data):
    
    list_of_centroids = list(zip(data.data[::2], data.data[1::2])) # convert list into list of tuples
    # here is the main part for the association
    pedestrians = ct.update(list_of_centroids)

    rospy.loginfo(pedestrians)
    #rospy.loginfo('+' * 100)



def centroids_listener():
    rospy.init_node('centriods_listener', anonymous = True)
    global ct
    ct = CentroidTracker()
    rospy.Subscriber('centroid2', Floatlist, callback)
    rospy.spin()
    
if __name__ == '__main__':
    centroids_listener()


