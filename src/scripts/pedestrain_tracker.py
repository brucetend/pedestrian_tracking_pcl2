#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 3 17:48:42 2019

@author: ag345
"""

import rospy
import numpy as np
from std_msgs.msg import String
from pedestrain_tracking.msg import Floatlist, Pedestrian_info, Pedestrians_array
from centroidtracker import CentroidTracker

def callback(data):
    
    list_of_centroids = list(zip(data.data[::2], data.data[1::2])) # convert list into list of tuples
    #Use the CentroidTracker class to do the association
    pedestrians_dict = ct.update(list_of_centroids)
    #Initialize p for a pedestrian
    p = Pedestrian_info()
    #Initialize an array for all pedestrians
    associated_pedestrians = Pedestrians_array()
    
    for key, value in pedestrians_dict.items():

        p.id = key
        p.x_last, p.y_last, p.x, p.y = value
        associated_pedestrians.data.append(p)
    
    #Pubilsh the position of all pedestrians as Pedestrians_array
    pub = rospy.Publisher('pedestrians_array', Pedestrians_array, queue_size=10)
    pub.publish(associated_pedestrians.data)
    rospy.loginfo(associated_pedestrians)




def centroids_listener():
    rospy.init_node('centriods_listener', anonymous = True)
    #Initialize the centroid tracker
    global ct
    ct = CentroidTracker()
    rospy.Subscriber('centroid2', Floatlist, callback)
    rospy.spin()
    
if __name__ == '__main__':
    centroids_listener()


