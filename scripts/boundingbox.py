#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 17:38:11 2019

@author: oh372
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def callback(data):
    #make a list of all points of incoming PointCloud2.msg but only relevant fields x,y,cluster,label
    cloud_points = list(point_cloud2.read_points(cloud=data, field_names = ('x','y','cluster','label')))
    
    
    dictionary={}
    for element in range(len(cloud_points)):
        data_point = list(cloud_points[element])
        
        # only proceed with points that have the label "pedestrian", 
        if data_point[3] == 3:
            key = data_point[2]

            #check if pedestrian already exists in dictionary by its cluster
            if key in dictionary:
                # append array of the key with new data points if key already exists in dictionary
                dictionary[key] = np.vstack([dictionary[key], [data_point[0], data_point[1]]])
                
            else:
                #make new entry in dictionary for new key
                dictionary.update({key: np.array([data_point[0], data_point[1]])})

    centroid_dictionary={}

    for cluster in dictionary:
        centroid = np.mean(dictionary[cluster], axis=0)
        centroid_dictionary.update({cluster: centroid})

    rospy.loginfo(centroid_dictionary)



def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('/cocar/ibeo_lux_201/cloud', PointCloud2, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()